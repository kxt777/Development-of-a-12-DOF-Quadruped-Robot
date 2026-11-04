import math
import struct
import unittest

from dm_imu.protocol import (
    ACCEL, GYRO, EULER, QUATERNION, HEADER, ImuFrame, ImuParser,
    SampleAssembler, crc16, euler_to_quaternion,
)


def packet(kind, values, slave=1):
    data = HEADER + bytes((slave, kind)) + struct.pack("<" + "f" * len(values), *values)
    return data + struct.pack("<H", crc16(data)) + b"\x0a"


class ImuProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_vendor_crc_coverage_includes_header(self):
        wire = packet(ACCEL, (0.0, 0.0, 9.81))
        self.assertEqual(len(wire), 19)
        self.assertEqual(crc16(wire[:16]), struct.unpack_from("<H", wire, 16)[0])
        legacy = wire[:16] + struct.pack("<H", crc16(wire[2:16])) + wire[-1:]
        parser = ImuParser()
        self.assertEqual(parser.append(legacy), [])
        self.assertEqual(parser.crc_errors, 1)

    def test_every_split_and_noise_prefix(self):
        wire = packet(ACCEL, (1.0, 2.0, 3.0)) + packet(GYRO, (0.1, 0.2, 0.3))
        for split in range(len(wire) + 1):
            parser = ImuParser()
            frames = parser.append(b"unrelated USB text\x55" + wire[:split])
            frames += parser.append(wire[split:])
            self.assertEqual([frame.kind for frame in frames], [ACCEL, GYRO], split)

    def test_crc_corruption_resynchronizes(self):
        bad = bytearray(packet(ACCEL, (1.0, 2.0, 3.0)))
        bad[8] ^= 1
        parser = ImuParser()
        good = packet(EULER, (10.0, 20.0, 30.0))
        frames = parser.append(bytes(bad) + good)
        self.assertEqual([frame.kind for frame in frames], [EULER])
        self.assertEqual(parser.crc_errors, 1)

    def test_variable_quaternion_length_and_normalization(self):
        parser = ImuParser()
        wire = packet(QUATERNION, (2.0, 0.0, 0.0, 0.0))
        self.assertEqual(len(wire), 23)
        assembler = SampleAssembler()
        assembler.add(ImuFrame(ACCEL, (0.0, 0.0, 9.81), 1), 0.0)
        assembler.add(ImuFrame(GYRO, (0.0, 0.0, 0.0), 1), 0.001)
        sample = assembler.add(parser.append(wire)[0], 0.002)
        self.assertEqual(sample.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(sample.orientation_source, "quaternion")

    def test_other_device_and_nonmeasurement_frames(self):
        parser = ImuParser()
        temperature = HEADER + b"\x01\x05" + struct.pack("<ffHH", 50.0, 30.0, 10, 0)
        temperature += struct.pack("<H", crc16(temperature)) + b"\x0a"
        frames = parser.append(packet(ACCEL, (1.0, 2.0, 3.0), slave=2) + temperature
                               + packet(GYRO, (4.0, 5.0, 6.0)))
        self.assertEqual([frame.kind for frame in frames], [GYRO])
        self.assertEqual(parser.other_devices, 1)
        self.assertEqual(parser.other_frames, 1)

    def test_nan_inf_and_zero_quaternion_rejected(self):
        parser = ImuParser()
        wire = packet(ACCEL, (math.nan, 0.0, 0.0))
        wire += packet(GYRO, (0.0, math.inf, 0.0))
        wire += packet(QUATERNION, (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(parser.append(wire), [])
        self.assertEqual(parser.format_errors, 3)

    def test_euler_converts_degrees_to_ros_quaternion(self):
        q = euler_to_quaternion((0.0, 0.0, 90.0))
        self.assertAlmostEqual(q[2], math.sqrt(0.5))
        self.assertAlmostEqual(q[3], math.sqrt(0.5))

    def test_complete_sample_preserves_si_vectors(self):
        assembler = SampleAssembler()
        self.assertIsNone(assembler.add(ImuFrame(ACCEL, (0.0, 0.0, 9.81), 1), 1.0))
        self.assertIsNone(assembler.add(ImuFrame(GYRO, (1.0, 2.0, 3.0), 1), 1.001))
        sample = assembler.add(ImuFrame(EULER, (0.0, 0.0, 0.0), 1), 1.002)
        self.assertEqual(sample.acceleration, (0.0, 0.0, 9.81))
        self.assertEqual(sample.angular_velocity, (1.0, 2.0, 3.0))
        self.assertEqual(sample.orientation_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertIsNone(assembler.sample(1.003))
        # An orientation update alone must not replay old accel/gyro readings.
        self.assertIsNone(assembler.add(ImuFrame(EULER, (1.0, 2.0, 3.0), 1), 1.004))

    def test_all_channels_must_update_between_samples(self):
        assembler = SampleAssembler()
        frames = [ImuFrame(ACCEL, (1.0, 2.0, 3.0), 1),
                  ImuFrame(GYRO, (4.0, 5.0, 6.0), 1),
                  ImuFrame(EULER, (0.0, 0.0, 0.0), 1)]
        for frame in frames:
            first = assembler.add(frame, 1.0)
        self.assertIsNotNone(first)
        self.assertIsNone(assembler.add(frames[0], 1.01))
        self.assertIsNone(assembler.add(frames[1], 1.01))
        self.assertIsNotNone(assembler.add(frames[2], 1.01))

    def test_stale_and_widely_separated_channels_not_combined(self):
        assembler = SampleAssembler(max_age=0.1, max_skew=0.03)
        assembler.add(ImuFrame(ACCEL, (0.0, 0.0, 9.81), 1), 0.0)
        assembler.add(ImuFrame(GYRO, (0.0, 0.0, 0.0), 1), 0.04)
        self.assertIsNone(assembler.add(ImuFrame(EULER, (0.0, 0.0, 0.0), 1), 0.05))
        self.assertIsNone(assembler.sample(0.2))

    def test_configuration_validation(self):
        for age, skew in [(0.0, 0.03), (0.1, 0.2), (math.inf, 0.03)]:
            with self.assertRaises(ValueError):
                SampleAssembler(age, skew)
        with self.assertRaises(ValueError):
            ImuParser(256)


if __name__ == "__main__":
    unittest.main()
