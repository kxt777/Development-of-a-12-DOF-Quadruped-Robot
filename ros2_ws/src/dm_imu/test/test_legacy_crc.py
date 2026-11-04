import struct
import unittest
from dm_imu.protocol import ImuParser,crc16,crc16_legacy_v1


class LegacyTests(unittest.TestCase):
    def test_independent_recorded_hardware_frame(self):
        # Captured passively on NUC, firmware V1.0.2.4, CAN/slave ID 0x0d.
        wire=bytes.fromhex('55aa0d015e5d0e3ece07343dd1c11bc15adf0a')
        self.assertEqual(crc16_legacy_v1(wire[:-3]),0xdf5a)
        p=ImuParser(13,'legacy_v1');frames=[]
        for byte in wire:frames.extend(p.append(bytes([byte])))
        self.assertEqual(len(frames),1)
        self.assertAlmostEqual(frames[0].values[2],-9.734818458557129)
        self.assertEqual(ImuParser(13).append(wire),[])

    def test_corrupted_packet_rejected_and_resync(self):
        wire=bytes.fromhex('55aa0d015e5d0e3ece07343dd1c11bc15adf0a')
        damaged=bytearray(wire);damaged[7]^=1
        p=ImuParser(13,'legacy_v1');frames=p.append(damaged+wire)
        self.assertEqual(p.crc_errors,1);self.assertEqual(len(frames),1)

    def test_algorithms_remain_distinct(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        self.assertEqual(crc16_legacy_v1(b'123456789'),0x5f16)


if __name__=='__main__':unittest.main(verbosity=2)
