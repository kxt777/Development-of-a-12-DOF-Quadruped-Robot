import math
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from dog_ik_walk.teleop import TeleopController,VelocityTable
from dog_ik_walk.heading import HeadingHold


class TeleopTests(unittest.TestCase):
    def make(self):
        c=TeleopController.__new__(TeleopController);c.teleop_lock=threading.RLock();c._lock=threading.Lock()
        c._enabled=True;c.stand_completed=True;c._disabling=False;c._commands=[];c._mode='hold';c._plan={'kind':'hold'}
        c._generation=1;c._cancel=threading.Event();c._shutdown=threading.Event();c._last_tick=1.
        c.drive=None;c.blocked_gesture=None;c.steering_mode=None;c.v=c.w=c.amplitude=0.;c.ramp=None;c.telemetry={}
        c.deployment={'teleop':{'max_forward_m_s':.08,'command_timeout_s':.5,'velocity_filter_seconds':.4},'heading':{'command_filter_seconds':.4},'walk_ramp_seconds':2.}
        c.velocity_table=SimpleNamespace(wmax=.15,frequency=1.2,sample=lambda *a:(np.zeros(12),np.zeros(12)))
        c.heading=HeadingHold(dict(kp=1.,kd=.15,command_filter_seconds=.4,trajectory_yaw_rate_rad_s=.15,max_imu_age_s=.1))
        c.heading.receive([1,0,0,0],[0,0,0],[0,0,-9.81],1.)
        c.kin=SimpleNamespace(stand_q=np.zeros(12));c.options={'max_control_gap':.2};c.last_command={}
        c._send_targets=lambda *a:None
        return c
    def command(self,seq=1,gesture='a',vx=.08,wz=0,active=True,stamp=1.01):return dict(seq=seq,gesture=gesture,vx=vx,wz=wz,active=active,issued_monotonic=stamp)
    def test_requires_stand_and_fresh_command(self):
        c=self.make();c.stand_completed=False
        with patch('dog_ik_walk.teleop.now',return_value=1.01):
            with self.assertRaises(RuntimeError):c.accept_velocity(self.command())
        c.stand_completed=True
        with patch('dog_ik_walk.teleop.now',return_value=2.):
            with self.assertRaises(ValueError):c.accept_velocity(self.command())
    def test_stream_does_not_reanchor_heading_and_turn_to_straight_does(self):
        c=self.make()
        with patch('dog_ik_walk.teleop.now',return_value=1.01):
            c.accept_velocity(self.command());c.accept_velocity(self.command(seq=2,vx=.04))
            self.assertEqual(c.heading.command_sequence,1)
            c.accept_velocity(self.command(seq=3,wz=.15));self.assertEqual(c.heading.command_sequence,2)
            c.w=.1;c.accept_velocity(self.command(seq=4));self.assertEqual(c.heading.rate,.1)
            self.assertEqual(c.heading.command_sequence,3)
    def test_release_does_not_resume_from_old_gesture(self):
        c=self.make()
        with patch('dog_ik_walk.teleop.now',return_value=1.01):
            c.accept_velocity(self.command());c.accept_velocity(self.command(seq=2,active=False))
            with self.assertRaises(ValueError):c.accept_velocity(self.command(seq=3))
            c.accept_velocity(self.command(seq=4,gesture='b'))
    def test_disconnect_settles_then_holds_without_new_packets(self):
        c=self.make()
        with patch('dog_ik_walk.teleop.now',return_value=1.01):c.accept_velocity(self.command(wz=.15))
        # Run real timeout/ramp logic with a harmless synthetic joint table.
        for t in np.arange(1.03,4.1,.02):
            with patch('dog_ik_walk.teleop.now',return_value=float(t)):c._tick(1,c._cancel)
            if c._plan['kind']=='hold':break
        self.assertEqual(c.blocked_gesture,'a');self.assertEqual(c._mode,'hold');self.assertEqual(c.amplitude,0)
    def test_table_wrap_continuity(self):
        table=VelocityTable.__new__(VelocityTable);table.count=128;table.frequency=1.2;table.vmax=.08;table.wmax=.15
        table.q=np.zeros((3,3,128,12))
        for k in range(128):table.q[:,:,k]=math.sin(2*math.pi*k/128)
        q1,_=table.sample(1.-1e-8,.03,-.07,.1,.1);q2,_=table.sample(1e-8,.03,-.07,.1,.1)
        self.assertLess(float(abs(q1-q2).max()),1e-6)


if __name__=='__main__':unittest.main()
