"""Offline equivalence, calibration and state-observation tests; no serial I/O."""
import unittest

import mujoco
import numpy as np

from dog_sim_bridge import (BodyState, DogController, DogMitCommand, SimControllerBridge,
                            StanceStateEstimator, feedback_to_model, native_mit_packet)
from dog_runtime import load_settings
from dog_protocol import RECORD


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config, cls.calibration = load_settings()

    def test_shared_command_producer_matches_original_with_identical_observations(self):
        bridge = SimControllerBridge('trot')
        data = mujoco.MjData(bridge.model)
        from dog_tradition.dog_kinematics import DogKinematics
        kin = DogKinematics(bridge.model)
        data.qpos[:7] = [0, 0, .191, 1, 0, 0, 0]
        data.qpos[kin.q_ids] = bridge.standing_q
        mujoco.mj_forward(bridge.model, data)
        original = DogController(bridge.model, data, bridge.config, bridge.standing_q)
        for tick in range(3500):
            mujoco.mj_step1(bridge.model, data)
            if tick % 5 == 0:
                state = BodyState(data.time, data.qpos[:3].copy(), data.qpos[3:7].copy(),
                                  data.qvel[:3].copy(), data.qvel[3:6].copy(),
                                  data.qpos[kin.q_ids].copy(), data.qvel[kin.v_ids].copy())
                command = bridge.step(state)
                original.update('trot')
                for key in ('q_des', 'dq_des', 'kp', 'kd', 'tau_ff'):
                    np.testing.assert_allclose(getattr(command, key), getattr(original.command, key),
                                               atol=2e-10, rtol=0)
            original.apply()
            mujoco.mj_step2(bridge.model, data)

    def test_native_fields_preserve_calibrated_positions_velocities_torques_and_gains(self):
        bridge = SimControllerBridge()
        q, dq, ff = bridge.standing_q, np.linspace(-1, 1, 12), np.linspace(-2, 2, 12)
        cmd = DogMitCommand(q, dq, np.full(12, 50.), np.ones(12), ff)
        packet = native_mit_packet(cmd, self.calibration, self.config['motor']['mapping_ranges'])
        for row in self.calibration:
            i = self.calibration.index(row)
            _, motor_id, _, p, v, kp, kd, t = RECORD.unpack_from(packet, 5+(row['can_id']-1)*RECORD.size)
            self.assertEqual(motor_id, row['can_id'])
            sign = row['direction']
            np.testing.assert_allclose([sign*(p-row['zero_rad']), sign*v, sign*t],
                                       [q[i], dq[i], ff[i]], atol=3e-7)
            self.assertEqual((kp, kd), (50., 1.))
            self.assertAlmostEqual(v*t, dq[i]*ff[i], places=6)

    def test_native_path_has_no_six_nm_pd_gate_but_does_not_silently_clip_wire_fields(self):
        command = DogMitCommand(np.full(12, .2), np.zeros(12), np.full(12, 50.),
                                np.ones(12), np.zeros(12))
        packet = native_mit_packet(command, self.calibration, self.config['motor']['mapping_ranges'])
        self.assertEqual(len(packet), 281)
        self.assertTrue(np.all(command.kp*command.q_des > 6))
        command.tau_ff[0] = 10.01
        with self.assertRaisesRegex(ValueError, 'wire mapping'):
            native_mit_packet(command, self.calibration, self.config['motor']['mapping_ranges'])

    def test_feedback_uses_inverse_mapping_for_each_motor_id(self):
        feedback = [dict(id=row['can_id'], position=row['zero_rad']+row['direction']*.15,
                         velocity=row['direction']*.3, torque=row['direction']*.7)
                    for row in reversed(self.calibration)]
        q, dq, torque = feedback_to_model(feedback, self.calibration)
        np.testing.assert_allclose(q, .15)
        np.testing.assert_allclose(dq, .3)
        np.testing.assert_allclose(torque, .7)

    def test_missing_mount_does_not_create_level_fake_observations(self):
        bridge = SimControllerBridge()
        with self.assertRaisesRegex(ValueError, 'not assumed'):
            StanceStateEstimator(bridge.model, None)

    def test_stance_estimator_respects_mount_and_no_slip_velocity(self):
        bridge = SimControllerBridge()
        mount = np.array([np.cos(.35), 0., 0., np.sin(.35)])
        estimator = StanceStateEstimator(bridge.model, mount)
        q = bridge.standing_q
        velocity = np.array([.02, -.03, .01])
        feet = estimator.kin.forward(q)
        dq = np.zeros(12)
        for leg in range(4):
            dq[3*leg:3*leg+3] = np.linalg.solve(estimator.kin.jacobian(leg), -velocity)
        state = estimator.update(0., q, dq, mount, np.zeros(3), np.ones(4, dtype=bool))
        np.testing.assert_allclose(state.velocity_world, velocity, atol=1e-12)
        np.testing.assert_allclose(state.quaternion_wxyz, [1, 0, 0, 0], atol=1e-12)
        self.assertAlmostEqual(state.position_world[2], .19, places=5)

    def test_body_angular_velocity_is_populated_in_mujoco_body_frame(self):
        bridge = SimControllerBridge()
        quat = np.array([np.cos(.2), 0, 0, np.sin(.2)])
        omega = np.array([.1, .2, .3])
        state = BodyState(0., np.array([0., 0., .19]), quat, np.zeros(3), omega,
                          bridge.standing_q, np.zeros(12))
        bridge.set_state(state)
        result = np.empty(6)
        mujoco.mj_objectVelocity(bridge.model, bridge.data, mujoco.mjtObj.mjOBJ_BODY,
                                bridge.model.body('root').id, result, 0)
        from dog_tradition.dog_model import rotation
        np.testing.assert_allclose(result[:3], rotation(quat)@omega, atol=1e-12)


if __name__ == '__main__':
    unittest.main(verbosity=2)
