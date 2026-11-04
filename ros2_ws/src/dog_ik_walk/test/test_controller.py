import unittest,copy,time,threading
import numpy as np
from dog_runtime import load_settings
from dog_link import DryRunLink,GLOBAL_ENABLE,GLOBAL_DISABLE
from dog_ik_walk.controller import IkWalkController,WriteGate,align_encoder_branches

class Tests(unittest.TestCase):
    def test_post_enable_pose_replaces_old_can_cache_before_gains(self):
        from dog_protocol import RECORD
        c,link=self.make(True);zero=c.calibration[5]['zero_rad']
        link._positions[5]=zero-1.34585  # Yesterday's cached standing position.
        measured=zero+2*np.pi-.0838254134
        original=link.send
        def reboot(packet,cancelled=None):
            original(packet,cancelled)
            if packet==GLOBAL_ENABLE:link._positions[5]=measured
        link.send=reboot
        try:
            c.start();deadline=time.monotonic()+1
            while not c.status()['connected'] and time.monotonic()<deadline:time.sleep(.01)
            c.request('enable');deadline=time.monotonic()+2
            while not c.status()['enabled'] and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(c.status()['enabled'],c.status()['message'])
            records=[RECORD.unpack_from(p,5+5*RECORD.size) for p in link.packets if p[:5]==b'DM12\x10']
            holding=[r for r in records if r[5]>0]
            self.assertTrue(holding)
            self.assertTrue(all(abs(r[3]-measured)<1e-5 for r in holding))
            self.assertEqual(c.enable_alignment['motors'][5]['branch_shift_turns'],1)
            self.assertAlmostEqual(c._q[5],.0838254134)
            self.assertAlmostEqual(c.calibration[5]['measurement_reference_motor_q_rad'],zero)
        finally:c.close()

    def test_branch_alignment_selects_nearest_zero_in_both_directions(self):
        for turns in [-1,0,1]:
            rows=[dict(id=6,position=-2.97+turns*2*np.pi-.1)]
            aligned,events=align_encoder_branches(rows,[dict(can_id=6,zero_rad=-2.97,direction=-1)],np.array([[-3.,3.]]))
            self.assertAlmostEqual(events[0]['model_position_rad'],.1)
            self.assertEqual(events[0]['branch_shift_turns'],turns)
        rows=[dict(id=6,position=-2.97+2*np.pi-.1)]
        aligned,_=align_encoder_branches(rows,[dict(can_id=6,zero_rad=-2.97,direction=-1)],np.array([[-3.,3.]]))
        again,events=align_encoder_branches(rows,aligned,np.array([[-3.,3.]]))
        self.assertAlmostEqual(again[0]['zero_rad'],aligned[0]['zero_rad'])
        self.assertAlmostEqual(again[0]['original_zero_rad'],-2.97)
        self.assertEqual(events[0]['branch_shift_turns'],1)

    def test_each_zero_reselects_nearest_branch_and_keeps_following_targets_consistent(self):
        c,link=self.make(True);c._enabled=True;link._enabled=True;c._cancel.clear()
        zero=c.calibration[5]['zero_rad'];link._positions[5]=zero+2*np.pi-.1
        try:
            c.move_to_pose(np.zeros(12),8.,'zero')
            self.assertAlmostEqual(c._plan['q0'][5],.1)
            self.assertAlmostEqual(c.calibration[5]['zero_rad'],zero+2*np.pi)
            self.assertEqual(c.zero_alignment['motors'][5]['branch_shift_turns'],1)
            # Next zero after feedback returns to the original encoder branch.
            link._positions[5]=zero+.2
            c.move_to_pose(np.zeros(12),8.,'zero')
            self.assertAlmostEqual(c._plan['q0'][5],-.2)
            self.assertAlmostEqual(c.calibration[5]['zero_rad'],zero)
        finally:c.close()

    def make(self,allow=False):
        config,calibration=load_settings();config=copy.deepcopy(config)
        config['runtime'].update(stand_seconds=.3,gain_ramp_seconds=.05)
        deployment={'body_height_m':.17,'walk_seconds':6.,'walk_ramp_seconds':2.}
        raw=DryRunLink([r['zero_rad'] for r in calibration]);gate=WriteGate(raw,allow)
        # Explicit permission applies only to the in-memory fake transport.
        config['allow_actuation']=allow
        return IkWalkController(config,calibration,deployment,gate),raw
    def test_four_beat_feasible_calibrated_trajectory(self):
        c,link=self.make();p=c.preflight
        self.assertEqual(p['min_planned_support_feet'],3)
        self.assertFalse(p['imu_used']);self.assertLess(p['max_ik_residual_m'],.001)
        np.testing.assert_allclose(c.walk_table[1][0],c.kin.stand_q,atol=3e-5)
        np.testing.assert_allclose(c.walk_table[1][-1],c.kin.stand_q,atol=3e-5)
        self.assertEqual(link.packets,[])
    def test_monitor_mode_cannot_write(self):
        c,link=self.make()
        with self.assertRaises(RuntimeError):c.link.send(GLOBAL_ENABLE)
        c.start();time.sleep(.05);c.close();self.assertEqual(link.packets,[])
    def test_uninitialized_can_cache_cannot_enable(self):
        c,link=self.make(True)
        original=link.snapshot
        def zero_cache():
            rows,stamp,count=original()
            for r in rows:
                for k in ('state','position','velocity','torque','tmos','tcoil'):r[k]=0
            return rows,stamp,count
        link.snapshot=zero_cache
        with self.assertRaisesRegex(RuntimeError,'uninitialized CAN cache'):c._feedback()
        self.assertEqual(link.packets,[])
    def test_mock_enable_stand_walk_disable(self):
        c,link=self.make(True)
        def wait(check,timeout=4):
            end=time.monotonic()+timeout
            while not check() and time.monotonic()<end:time.sleep(.01)
            self.assertTrue(check(),c.status())
        try:
            c.start();wait(lambda:c.status()['connected']);c.request('enable');wait(lambda:c.status()['enabled'])
            c.request('stand');wait(lambda:c.stand_completed)
            c.request('walk');wait(lambda:c.status()['mode']=='walk_done',8)
            self.assertGreater(c.sequence,100)
            self.assertTrue(all(all(x==0 for x in r['tau_ff']) for r in c.history))
            c.disable('test');wait(lambda:c.status()['disable_state']=='confirmed')
            self.assertIn(GLOBAL_DISABLE,link.packets)
        finally:c.close()
    def test_zero_stand_walk_sequence_finishes_holding(self):
        from dog_ik_walk.sequence import run_sequence
        c,link=self.make(True)
        deployment=dict(c.deployment,zero_seconds=.3,zero_hold_seconds=.1,stand_seconds=.3,hold_seconds=.1)
        report={};stages=[]
        def record_sleep(seconds):
            stages.append(report.get('stage'));time.sleep(seconds)
        try:
            c.start();time.sleep(.1)
            run_sequence(c,deployment,report,threading.Event(),sleep=record_sleep)
            self.assertEqual(report['state'],'command_sequence_completed_holding_stand')
            self.assertTrue(c.status()['enabled'])
            self.assertNotIn(GLOBAL_DISABLE,link.packets)
            self.assertEqual(list(dict.fromkeys(stages)),['enable','zero','zero_hold','stand','stand_hold','walk'])
            self.assertGreaterEqual(report['walk_finished_monotonic']-report['walk_start_monotonic'],6.)
            np.testing.assert_allclose(c._q,c.kin.stand_q)
        finally:c.disable('test complete');time.sleep(.1);c.close()
if __name__=='__main__':unittest.main()
