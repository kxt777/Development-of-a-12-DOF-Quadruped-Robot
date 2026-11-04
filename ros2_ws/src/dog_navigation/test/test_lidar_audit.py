import struct
from types import SimpleNamespace as Obj
import unittest
from dog_navigation.lidar_audit import StreamStats, inspect_cloud
from dog_navigation.lidar_config import make_config


class AuditTest(unittest.TestCase):
    def test_stamps_and_end_silence_are_distinct(self):
        stats = StreamStats()
        for rx, stamp in [(10, 100), (11, 100), (13, 90)]:
            stats.observe(rx, stamp, 200, 'dog_lidar_link')
        report = stats.report(17)
        self.assertAlmostEqual(report['received_hz'], 2 / 3)
        self.assertEqual(report['max_interarrival_gap_s'], 2)
        self.assertEqual(report['silence_at_end_s'], 4)
        self.assertEqual(report['repeated_stamps'], 1)
        self.assertEqual(report['backward_stamps'], 1)
        self.assertIsNone(StreamStats().report(17)['received_hz'])

    def test_big_endian_padded_cloud_and_truncation(self):
        fields = [Obj(name=k, offset=i * 4, datatype=7, count=1)
                  for i, k in enumerate(('x', 'y', 'z'))]
        cloud = Obj(width=1, height=2, point_step=12, row_step=16,
                    is_bigendian=True, fields=fields,
                    data=struct.pack('>fff4xfff4x', 1, 2, 3, 4, float('nan'), 6))
        result = inspect_cloud(cloud)
        self.assertTrue(result['layout_valid'])
        self.assertEqual(result['sampled_xyz'], 2)
        self.assertEqual(result['nonfinite_xyz'], 1)
        cloud.data = cloud.data[:-1]
        self.assertFalse(inspect_cloud(cloud)['layout_valid'])

    def test_network_configuration_does_not_guess_mounting(self):
        config = make_config('192.168.1.5', '192.168.1.156', 'mid360')
        self.assertEqual(config['MID360']['host_net_info']['point_data_port'], 56301)
        self.assertEqual(config['lidar_configs'][0]['extrinsic_parameter']['yaw'], 0)
        with self.assertRaises(ValueError):
            make_config('192.168.1.5', '192.168.1.5')
        config_s = make_config('192.168.1.5', '192.168.1.12', 'mid360s')
        self.assertEqual(config_s['Mid360s']['host_net_info'][0]['host_ip'], '192.168.1.5')


if __name__ == '__main__':
    unittest.main()
