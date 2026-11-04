"""Offline checks only; tests never import or open serial transports."""
import copy
import math
import struct
import unittest

import numpy as np

from dog_demo import load_config, preview
from dog_gait import DogGait
from dog_imu import ACCEL, GYRO, QUATERNION, ImuParser, SampleAssembler, crc16
from dog_kinematics import DogKinematics
from dog_protocol import FeedbackParser, MAGIC, RECORD, PACKET_SIZE, encode_mit, model_to_motor


class DogTests(unittest.TestCase):
    def test_stand_geometry(self):
        kin = DogKinematics()
        expected = [[.145, .13, -.19], [.145, -.13, -.19],
                    [-.145, .13, -.19], [-.145, -.13, -.19]]
        np.testing.assert_allclose(kin.forward(kin.stand_q), expected, atol=3e-5)

    def test_jacobian_finite_difference(self):
        kin = DogKinematics()
        rng = np.random.default_rng(27)
        for _ in range(8):
            q = kin.stand_q + rng.uniform(-.1, .1, 12)
            _, jac = kin.evaluate(q)
            for leg in range(4):
                for j in range(3):
                    step = np.zeros(12)
                    step[3*leg+j] = 1e-6
                    numerical = (kin.forward(q+step)[leg]-kin.forward(q-step)[leg])/2e-6
                    np.testing.assert_allclose(jac[leg, :, j], numerical, atol=1e-8)

    def test_ik_round_trip(self):
        kin = DogKinematics()
        wanted = kin.stand_q + np.tile([.04, -.04, .06], 4)
        q, errors = kin.solve(kin.forward(wanted), kin.stand_q)
        self.assertLess(errors.max(), 2e-5)
        np.testing.assert_allclose(q, wanted, atol=.001)

    def test_diagonal_phase(self):
        kin = DogKinematics()
        c = load_config()['sim_candidate']
        gait = DogGait(c, kin.forward(kin.stand_q), 'trot')
        # After ramp, FL+BR swing together and FR+BL remain in stance.
        elapsed = (10+.8)/gait.config['frequency']
        feet = gait.targets(c['settle_seconds']+elapsed)
        lift = feet[:, 2]-gait.neutral[:, 2]
        np.testing.assert_allclose(lift[[0, 3]], lift[0], atol=1e-12)
        self.assertGreater(lift[0], .005)
        np.testing.assert_allclose(lift[[1, 2]], 0, atol=1e-12)

    def test_mit_encoder(self):
        wire = encode_mit({8: (.3, .1, 50., 1., .2)})
        self.assertEqual(len(wire), 281)
        self.assertEqual(wire[:5], b'DM12\x10')
        self.assertEqual(RECORD.unpack_from(wire, 5)[0], 0)
        row = RECORD.unpack_from(wire, 5+7*RECORD.size)
        self.assertEqual(row[:3], (1, 8, 1))
        np.testing.assert_allclose(row[3:], (.3, .1, 50., 1., .2), atol=1e-6)
        for records in ({0: (0,)*5}, {1: (math.nan, 0, 1, 1, 0)}, {1: (0, 0, 1, 6, 0)}):
            with self.assertRaises(ValueError):
                encode_mit(records)

    def test_feedback_resync(self):
        wire = MAGIC+b'\x11'+b''.join(RECORD.pack(1, i, 0, .1*i, .2, .3, 25., 26.)
                                        for i in range(1, 13))
        parser = FeedbackParser()
        self.assertEqual(parser.append(b'noise'+wire[:17]), [])
        frames = parser.append(wire[17:]+wire)
        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0][7]['id'], 8)
        bad = bytearray(wire)
        bad[6] = 9
        self.assertEqual(len(FeedbackParser().append(bytes(bad)+wire)), 1)
        self.assertEqual(PACKET_SIZE, len(wire))

    def test_imu_parser_and_assembler(self):
        def wire(kind, values):
            payload = b'\x55\xaa\x01'+bytes([kind])+struct.pack('<'+'f'*len(values), *values)
            return payload+struct.pack('<H', crc16(payload))+b'\x0a'
        data = wire(ACCEL, (0, 0, 9.81))+wire(GYRO, (0, 0, 0))+wire(QUATERNION, (1, 0, 0, 0))
        parser = ImuParser()
        self.assertEqual(parser.append(b'noise'+data[:7]), [])
        frames = parser.append(data[7:])
        self.assertEqual(len(frames), 3)
        assembly = SampleAssembler()
        sample = None
        for frame in frames:
            sample = assembly.add(frame, 1.)
        self.assertEqual(sample.orientation_xyzw, (0., 0., 0., 1.))
        self.assertIsNone(assembly.sample(1.))
        corrupt = bytearray(data[:19])
        corrupt[5] ^= 1
        self.assertEqual(ImuParser().append(corrupt), [])

    def test_unconfirmed_mapping_rejected(self):
        from dog_runtime import load_settings
        calibration = copy.deepcopy(load_settings()[1])
        for row in calibration:
            row.update(direction=None, zero_rad=None)
        with self.assertRaises(ValueError):
            model_to_motor(np.zeros(12), np.zeros(12), np.zeros(12), calibration)
        calibrated = copy.deepcopy(calibration)
        for row in calibrated:
            row.update(direction=-1, zero_rad=.4)
        result = model_to_motor(np.ones(12)*.1, np.ones(12)*.2, np.ones(12)*.3, calibrated)
        np.testing.assert_allclose(result[0], (.3, -.2, -.3))

    def test_mode_previews(self):
        config = load_config()
        for mode in ('stand', 'trot', 'walk'):
            result = preview(config, mode, 6.)
            self.assertLess(result['max_ik_error_m'], 2e-5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
