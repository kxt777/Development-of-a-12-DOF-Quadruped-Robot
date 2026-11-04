"""ROS callback/sequence regression tests; fake controller, no serial I/O."""
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from dog_ik_walk.node import IkWalkNode
from dog_ik_walk.heading import HeadingHold
from dog_ik_walk.sequence import run_sequence

SETTINGS=dict(kp=1.,kd=.15,command_filter_seconds=.4,trajectory_yaw_rate_rad_s=.15,max_imu_age_s=.1)

def receive(h,yaw,t):
    h.receive([math.cos(yaw/2),math.sin(yaw/2),0.,0.],[0,0,0],[0,0,-9.81],t)

class CommandReceiptTests(unittest.TestCase):
    def make_node(self):
        h=HeadingHold(SETTINGS);receive(h,.4,1.)
        return SimpleNamespace(allow=True,imu_used=True,controller=SimpleNamespace(heading=h,request=Mock()),
            demo_thread=None,demo_cancel=threading.Event(),run_demo=Mock(),namespace='/dog_ik_walk_imu')

    def test_demo_latches_before_background_preparation(self):
        n=self.make_node()
        with patch('dog_ik_walk.node.time.monotonic',return_value=1.02),patch('dog_ik_walk.node.threading.Thread') as worker:
            worker.return_value.start.side_effect=lambda:self.assertAlmostEqual(n.controller.heading.reference,.4)
            response=IkWalkNode.on_action(n,'demo',SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(n.controller.heading.command['received_monotonic'],1.02)
        self.assertEqual(n.controller.heading.command['source'],'demo')

    def test_direct_walk_passes_callback_receipt_time(self):
        n=self.make_node()
        with patch('dog_ik_walk.node.time.monotonic',return_value=1.02):
            response=IkWalkNode.on_action(n,'walk',SimpleNamespace())
        self.assertTrue(response.success)
        n.controller.request.assert_called_once_with('walk',command_received=1.02)

    def test_direct_turn_queues_only_walk_and_commits_direction(self):
        for action in ('walk_left','walk_right'):
            n=self.make_node();h=n.controller.heading
            n.controller.request.side_effect=lambda name,command_received:h.activate_command(h.capture_command(name,command_received))
            with patch('dog_ik_walk.node.time.monotonic',return_value=1.02):
                response=IkWalkNode.on_action(n,action,SimpleNamespace())
            self.assertTrue(response.success)
            n.controller.request.assert_called_once_with('walk',command_received=1.02)
            self.assertEqual(h.command['source'],action)
            rate,_=h.step(.02,1.03)
            self.assertGreater(rate*(1 if action=='walk_left' else -1),0)
            self.assertFalse(n.demo_cancel.is_set())

    def test_busy_demo_and_monitor_mode_do_not_capture_direction(self):
        for monitor in [False,True]:
            n=self.make_node();n.allow=not monitor
            if not monitor:n.demo_thread=Mock();n.demo_thread.is_alive.return_value=True
            with patch('dog_ik_walk.node.time.monotonic',return_value=1.02):
                response=IkWalkNode.on_action(n,'demo',SimpleNamespace())
            self.assertFalse(response.success)
            self.assertIsNone(n.controller.heading.reference)

    def test_demo_sequence_keeps_t1_after_zero_and_stand(self):
        h=HeadingHold(SETTINGS);receive(h,.4,1.);h.activate_command(h.capture_command('demo',1.02))
        timer=[1.02];calls=[]
        class Controller:
            options={'gain_ramp_seconds':2.};stand_completed=False;_lock=threading.Lock();_plan={'seconds':8.}
            enabled=False;mode=None
            def status(self):return dict(enabled=self.enabled,mode=self.mode,disabling=False,disable_state='idle')
            def _feedback(self,**kwargs):return []
            def _model_positions(self,rows):return np.zeros(12)
            def move_to_pose(self,*args):self.mode='hold'
            def request(self,action,**kwargs):
                calls.append((action,kwargs))
                if action=='enable':self.enabled=True
                if action=='stand':
                    receive(h,.7,timer[0]);self.stand_completed=True
                if action=='walk':
                    self_test.assertTrue(kwargs['preserve_heading'])
                    self_test.assertAlmostEqual(h.reference,.4)
                    self.mode='walk_done'
        self_test=self
        def sleep(dt):timer[0]+=dt
        report={};deployment=dict(imu_used=True,zero_seconds=8.,zero_hold_seconds=3.,stand_seconds=8.,hold_seconds=3.,walk_seconds=20.)
        run_sequence(Controller(),deployment,report,threading.Event(),clock=lambda:timer[0],sleep=sleep)
        self.assertEqual(report['state'],'command_sequence_completed_holding_stand')
        self.assertEqual(h.command['sequence'],1)
        self.assertEqual(calls[-1],('walk',{'preserve_heading':True}))

if __name__=='__main__':unittest.main(verbosity=2)
