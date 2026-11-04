"""Response requirements derived from the October 7 hardware log; no serial I/O."""
import math
import unittest
import numpy as np
from dog_ik_walk.heading import HeadingHold


SETTINGS=dict(kp=1.,kd=.15,command_filter_seconds=.4,trajectory_yaw_rate_rad_s=.15,max_imu_age_s=.1,
    straight_response=dict(full_correction_error_deg=5.,braking_error_deg=3.,
        yaw_rate_filter_seconds=1/(2*math.pi*1.2),command_filter_seconds=.08))


def receive(h,yaw_deg,rate_deg,stamp):
    yaw=math.radians(yaw_deg)
    h.receive([math.cos(yaw/2),math.sin(yaw/2),0.,0.],
        [0.,0.,-math.radians(rate_deg)],[0.,0.,-9.81],stamp)


def controller(source='walk',settings=None):
    h=HeadingHold(SETTINGS if settings is None else settings)
    receive(h,0,0,1.);h.activate_command(h.capture_command(source,1.))
    return h


class ResponseTests(unittest.TestCase):
    def test_three_degrees_responds_even_with_large_opposing_gyro(self):
        for direction in (-1,1):
            h=controller()
            for i in range(1,6):
                # Body is already returning; a large instantaneous gait swing
                # must not cancel correction while still >=3 degrees off-line.
                receive(h,-direction*3,direction*60,1+i*.02)
                rate,_=h.step(.02,1+i*.02)
            self.assertGreater(direction*rate,.06)
            self.assertAlmostEqual(h.detail['yaw_rate_requested_rad_s'],direction*.09)

    def test_five_degrees_uses_tested_turn_bank_quickly(self):
        h=controller()
        for i in range(1,6):
            receive(h,-5,0,1+i*.02);rate,_=h.step(.02,1+i*.02)
        self.assertGreater(rate,.1)  # >2/3 bank in 100 ms, not a physical yaw claim.
        self.assertEqual(h.detail['yaw_rate_requested_rad_s'],.15)

    def test_no_three_degree_deadband_and_trend_brakes_near_target(self):
        h=controller();receive(h,-1,0,1.02);self.assertGreater(h.step(.02,1.02)[0],0)
        h=controller();receive(h,-1,20,1.02);h.step(.02,1.02)
        self.assertLess(h.detail['yaw_rate_requested_rad_s'],0)
        self.assertGreater(h.detail['braking_weight'],0)

    def test_missing_samples_fade_and_recovery_keeps_t1(self):
        h=controller();receive(h,-5,50,1.02);before=h.step(.02,1.02)[0]
        after=h.step(.02,1.3)[0]
        self.assertGreater(after,0);self.assertLess(after,before)
        self.assertIsNone(h.filtered_yaw_rate)
        receive(h,-4,0,2.);h.step(.02,2.)
        self.assertAlmostEqual(h.reference,0);self.assertEqual(h.command['sequence'],1)
        self.assertEqual(h.filtered_yaw_rate,0)

    def test_manual_turns_are_identical_to_existing_response(self):
        legacy={k:v for k,v in SETTINGS.items() if k!='straight_response'}
        for source in ('demo_right','demo_left','walk_right','walk_left'):
            old=controller(source,legacy);new=controller(source)
            for i in range(1,101):
                for h in (old,new):receive(h,i*.1,20,1+i*.02)
                np.testing.assert_allclose(old.step(.02,1+i*.02),new.step(.02,1+i*.02),atol=0,rtol=0)

    def test_new_command_resets_trend_and_stays_within_existing_bank(self):
        h=controller()
        for i in range(1,21):
            receive(h,-15 if i%2 else 15,100 if i%2 else -100,1+i*.02)
            rate,derivative=h.step(.02,1+i*.02)
            self.assertLessEqual(abs(rate),.15)
            self.assertLessEqual(abs(derivative),3.+1e-12)
        h.activate_command(h.capture_command('walk',1.4));self.assertIsNone(h.filtered_yaw_rate)
        self.assertEqual(h.rate,0);self.assertEqual(h.command['sequence'],2)

    def test_invalid_response_configuration_is_rejected(self):
        for value in (0,float('nan'),float('inf')):
            cfg={**SETTINGS,'straight_response':{**SETTINGS['straight_response'],'command_filter_seconds':value}}
            with self.assertRaises(ValueError):HeadingHold(cfg)


if __name__=='__main__':unittest.main(verbosity=2)
