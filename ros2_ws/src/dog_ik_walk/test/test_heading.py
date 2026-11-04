import math
import unittest
from unittest.mock import patch
import numpy as np
from dog_ik_walk.heading import HeadingHold,mounted_sample,rotation_xyzw
from dog_ik_walk.imu_controller import foot_targets,ImuWalkController
from dog_ik_walk.controller import IkWalkController


SETTINGS=dict(kp=1.,kd=.15,command_filter_seconds=.25,trajectory_yaw_rate_rad_s=.15,max_imu_age_s=.1)


def sensor_quaternion(body_yaw):
    return [math.cos(body_yaw/2),math.sin(body_yaw/2),0.,0.]


class HeadingTests(unittest.TestCase):
    def test_explicit_left_turn_is_mirror_of_right(self):
        controllers=[]
        for source in ('demo_left','demo_right'):
            h=HeadingHold(SETTINGS);h.receive(sensor_quaternion(0),[0,0,0],[0,0,-9.81],1.)
            h.activate_command(h.capture_command(source,1.));controllers.append(h)
        for i in range(1,101):
            stamp=1+i*.02
            for h,sign in zip(controllers,(1,-1)):
                h.receive(sensor_quaternion(sign*i*.01),[0,0,-sign*.15],[0,0,-9.81],stamp)
                rate,_=h.step(.02,stamp)
                self.assertGreater(sign*rate,0)
            self.assertAlmostEqual(controllers[0].rate,-controllers[1].rate)
        self.assertEqual(controllers[0].detail['control_mode'],'fixed_left_turn')

    def test_explicit_right_turn_does_not_countersteer_back_to_start(self):
        h=HeadingHold(SETTINGS);h.receive(sensor_quaternion(0),[0,0,0],[0,0,-9.81],1.)
        h.activate_command(h.capture_command('demo_right',1.))
        for i in range(1,101):
            stamp=1+i*.02
            h.receive(sensor_quaternion(-i*.01),[0,0,.15],[0,0,-9.81],stamp)
            rate,_=h.step(.02,stamp)
            self.assertLess(rate,0);self.assertGreaterEqual(rate,-.15)
        self.assertAlmostEqual(rate,-.15*(1-(.25/.27)**100))
        self.assertIsNone(h.detail['heading_error_rad'])
        h.step(.02,4.);self.assertFalse(h.detail['imu_available'])
        h.receive(sensor_quaternion(-1.),[0,0,0],[0,0,-9.81],5.)
        h.activate_command(h.capture_command('demo',5.))
        h.receive(sensor_quaternion(-1.1),[0,0,0],[0,0,-9.81],5.02)
        self.assertGreater(h.step(.02,5.02)[0],0)

    def test_missing_imu_does_not_abort_or_reanchor_and_recovers(self):
        h=HeadingHold(SETTINGS);h.receive(sensor_quaternion(0),[0,0,0],[0,0,-9.81],1.);h.reset(1.)
        h.receive(sensor_quaternion(-.1),[0,0,0],[0,0,-9.81],1.02)
        before=h.step(.02,1.03)[0]
        missing=h.step(.02,1.3)[0]
        self.assertGreater(missing,0);self.assertLess(missing,before)
        self.assertFalse(h.detail['imu_available']);self.assertEqual(h.gap_count,1)
        h.step(.02,1.4);self.assertEqual(h.gap_count,1)
        self.assertAlmostEqual(h.reference,0)
        h.receive(sensor_quaternion(.1),[0,0,0],[0,0,-9.81],2.)
        h.step(.02,2.01)
        self.assertTrue(h.detail['imu_available'])
        self.assertAlmostEqual(h.detail['heading_error_rad'],-.1)
        self.assertEqual(h.command['sequence'],1)

    def test_command_heading_survives_preparation_and_next_command_reanchors(self):
        h=HeadingHold(SETTINGS)
        h.receive(sensor_quaternion(.4),[0,0,0],[0,0,-9.81],1.)
        h.activate_command(h.capture_command('demo',1.02))
        # The body turns during zero/stand preparation; this is not a new command.
        h.receive(sensor_quaternion(.7),[0,0,0],[0,0,-9.81],25.)
        rate,_=h.step(.02,25.01)
        self.assertAlmostEqual(h.reference,.4)
        self.assertAlmostEqual(h.detail['heading_error_rad'],-.3)
        self.assertLess(rate,0)
        self.assertEqual(h.command['received_monotonic'],1.02)
        h.activate_command(h.capture_command('walk',25.02))
        self.assertAlmostEqual(h.reference,.7)
        self.assertEqual(h.command['sequence'],2)
        h.receive(sensor_quaternion(.6),[0,0,0],[0,0,-9.81],25.03)
        self.assertGreater(h.step(.02,25.04)[0],0)

    def test_walk_queue_and_start_do_not_recapture_heading(self):
        c=ImuWalkController.__new__(ImuWalkController);c.heading=HeadingHold(SETTINGS)
        c.heading.receive(sensor_quaternion(.4),[0,0,0],[0,0,-9.81],1.)
        with patch.object(IkWalkController,'request') as enqueue:
            c.request('walk',command_received=1.02)
            enqueue.assert_called_once_with('walk')
        c.heading.receive(sensor_quaternion(.7),[0,0,0],[0,0,-9.81],2.)
        with patch.object(IkWalkController,'_begin_motion'):
            c._begin_motion('walk',0,None)
        self.assertAlmostEqual(c.heading.reference,.4)
        with patch.object(IkWalkController,'request'):
            c.request('walk',preserve_heading=True)
        self.assertAlmostEqual(c.heading.reference,.4)
        self.assertEqual(c.heading.command['sequence'],1)

    def test_rejected_and_stale_commands_do_not_replace_reference(self):
        c=ImuWalkController.__new__(ImuWalkController);c.heading=HeadingHold(SETTINGS)
        c.heading.receive(sensor_quaternion(.4),[0,0,0],[0,0,-9.81],1.)
        c.heading.reset(1.)
        c.heading.receive(sensor_quaternion(.7),[0,0,0],[0,0,-9.81],2.)
        with patch.object(IkWalkController,'request',side_effect=RuntimeError('not connected')):
            with self.assertRaises(RuntimeError):c.request('walk',command_received=2.01)
        with patch.object(IkWalkController,'request') as enqueue:
            with self.assertRaises(RuntimeError):c.request('walk',command_received=3.)
            enqueue.assert_not_called()
        self.assertAlmostEqual(c.heading.reference,.4)
        self.assertEqual(c.heading.command['sequence'],1)

    def test_internal_walk_without_external_command_is_rejected(self):
        c=ImuWalkController.__new__(ImuWalkController);c.heading=HeadingHold(SETTINGS)
        with patch.object(IkWalkController,'request') as enqueue:
            with self.assertRaises(RuntimeError):c.request('walk',preserve_heading=True)
            enqueue.assert_not_called()

    def test_inverted_mount_and_gyro(self):
        sample=mounted_sample(sensor_quaternion(.2),[0.,0.,-.3],[0.,0.,-9.81])
        self.assertAlmostEqual(sample['yaw'],.2)
        self.assertAlmostEqual(sample['yaw_rate'],.3)
        np.testing.assert_allclose(sample['accel_body'],[0.,0.,9.81])
        np.testing.assert_allclose(rotation_xyzw(sample['quaternion_xyzw']),
                                   rotation_xyzw(sensor_quaternion(.2))@np.diag([1.,-1.,-1.]),atol=1e-12)

    def test_right_error_commands_left_and_reverse(self):
        for yaw,sign in [(-.1,1),(.1,-1)]:
            h=HeadingHold(SETTINGS);h.receive(sensor_quaternion(0.),[0,0,0],[0,0,-9.81],1.);h.reset(1.)
            h.receive(sensor_quaternion(yaw),[0,0,0],[0,0,-9.81],1.01)
            rate,_=h.step(.02,1.02);self.assertGreater(rate*sign,0)

    def test_heading_wrap(self):
        h=HeadingHold(SETTINGS);h.receive(sensor_quaternion(math.radians(179)),[0,0,0],[0,0,-9.81],1.);h.reset(1.)
        h.receive(sensor_quaternion(math.radians(-179)),[0,0,0],[0,0,-9.81],1.01);h.step(.02,1.02)
        self.assertAlmostEqual(h.detail['heading_error_rad'],math.radians(-2))

    def test_stale_and_invalid_orientation_not_used(self):
        h=HeadingHold(SETTINGS)
        with self.assertRaises(RuntimeError):h.reset(1.)
        h.receive(sensor_quaternion(0.),[0,0,0],[0,0,-9.81],1.)
        with self.assertRaises(RuntimeError):h.reset(1.2)
        with self.assertRaises(ValueError):h.receive([0,0,0,0],[0,0,0],[0,0,-9.81])

    def test_positive_turn_outer_feet_advance_more(self):
        n=np.array([[.16,.13,-.17],[.16,-.13,-.17],[-.16,.13,-.17],[-.16,-.13,-.17]])
        p=dict(frequency=1.,duty_factor=.8,ramp_seconds=2.,phase_offsets=[0.]*4,step_height=.025,forward_speed=.075)
        straight=foot_targets(n,p,4.,20.,0.);left=foot_targets(n,p,4.,20.,.15)
        self.assertGreater(left[1,0]-straight[1,0],0.)
        self.assertLess(left[0,0]-straight[0,0],0.)
        self.assertGreater(left[0,1]-straight[0,1],0.)


if __name__=='__main__':unittest.main(verbosity=2)
