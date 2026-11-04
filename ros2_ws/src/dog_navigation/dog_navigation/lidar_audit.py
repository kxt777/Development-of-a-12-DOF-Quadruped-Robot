"""Read-only bounded PointCloud2/IMU audit. Never publishes motor or navigation commands."""
import argparse
import json
import math
from pathlib import Path
import struct
import time


class StreamStats:
    def __init__(self):
        self.count = 0
        self.first_rx = self.last_rx = self.last_stamp = None
        self.max_gap_s = 0.0
        self.repeated_stamps = self.backward_stamps = 0
        self.zero_stamps = 0
        self.min_age_s = self.max_age_s = None
        self.frames = set()

    def observe(self, received_s, stamp_ns, now_ros_ns, frame):
        if self.last_rx is not None:
            self.max_gap_s = max(self.max_gap_s, received_s - self.last_rx)
        if self.last_stamp is not None:
            self.repeated_stamps += int(stamp_ns == self.last_stamp)
            self.backward_stamps += int(stamp_ns < self.last_stamp)
        if self.first_rx is None:
            self.first_rx = received_s
        self.last_rx, self.last_stamp = received_s, stamp_ns
        self.count += 1
        self.zero_stamps += int(stamp_ns == 0)
        age = (now_ros_ns - stamp_ns) / 1e9
        self.min_age_s = age if self.min_age_s is None else min(self.min_age_s, age)
        self.max_age_s = age if self.max_age_s is None else max(self.max_age_s, age)
        self.frames.add(frame)

    def report(self, finish_s):
        span = None if self.count < 2 else self.last_rx - self.first_rx
        return dict(
            messages=self.count,
            received_hz=(self.count - 1) / span if span and span > 0 else None,
            max_interarrival_gap_s=self.max_gap_s if self.count > 1 else None,
            silence_at_end_s=finish_s - self.last_rx if self.last_rx is not None else None,
            repeated_stamps=self.repeated_stamps, backward_stamps=self.backward_stamps,
            zero_stamps=self.zero_stamps,
            min_stamp_age_s=self.min_age_s, max_stamp_age_s=self.max_age_s,
            frame_ids=sorted(self.frames),
        )


def inspect_cloud(msg):
    """Inspect buffer layout and a bounded XYZ sample, honoring organized-cloud padding."""
    fields = {f.name: f for f in msg.fields}
    result = dict(points=msg.width * msg.height,
                  fields=sorted(fields), sampled_xyz=0, nonfinite_xyz=0,
                  layout_valid=False)
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        return result
    if len(msg.data) < msg.row_step * msg.height:
        return result
    if any(k not in fields for k in ('x', 'y', 'z')):
        return result
    formats = {7: 'f', 8: 'd'}  # sensor_msgs/PointField FLOAT32 and FLOAT64
    readers = []
    for key in ('x', 'y', 'z'):
        field = fields[key]
        if field.datatype not in formats or field.count != 1:
            return result
        reader = struct.Struct(('>' if msg.is_bigendian else '<') + formats[field.datatype])
        if field.offset < 0 or field.offset + reader.size > msg.point_step:
            return result
        readers.append((field.offset, reader))
    result['layout_valid'] = True
    total = result['points']
    stride = max(1, (total + 1023) // 1024)
    for index in range(0, total, stride):
        row, col = divmod(index, msg.width)
        base = row * msg.row_step + col * msg.point_step
        xyz = [reader.unpack_from(msg.data, base + offset)[0] for offset, reader in readers]
        result['sampled_xyz'] += 1
        result['nonfinite_xyz'] += int(not all(math.isfinite(v) for v in xyz))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--format', choices=('pointcloud2', 'custom'), default='pointcloud2')
    parser.add_argument('--output', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if args.output.exists():
        parser.error('Output already exists; use a new report path')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, Imu
    rclpy.init(args=ros_args)
    node = Node('dog_lidar_audit')
    stats = {'pointcloud': StreamStats(), 'imu': StreamStats()}
    detail = dict(empty_clouds=0, malformed_clouds=0, sampled_xyz=0,
                  nonfinite_xyz=0, nonfinite_imu=0, min_points=None, max_points=0)

    def observe(kind, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stats[kind].observe(time.monotonic(), stamp, node.get_clock().now().nanoseconds,
                            msg.header.frame_id)

    def cloud_callback(msg):
        observe('pointcloud', msg)
        info = inspect_cloud(msg)
        detail['last_point_fields'] = info['fields']
        n = info['points']
        detail['empty_clouds'] += int(n == 0)
        detail['min_points'] = n if detail['min_points'] is None else min(n, detail['min_points'])
        detail['max_points'] = max(n, detail['max_points'])
        detail['malformed_clouds'] += int(not info['layout_valid'])
        for key in ('sampled_xyz', 'nonfinite_xyz'):
            detail[key] += info[key]

    def imu_callback(msg):
        observe('imu', msg)
        values = [getattr(v, axis) for v in (msg.angular_velocity, msg.linear_acceleration)
                  for axis in ('x', 'y', 'z')]
        detail['nonfinite_imu'] += int(not all(math.isfinite(v) for v in values))

    def custom_callback(msg):
        observe('pointcloud', msg)
        n = len(msg.points)
        detail['empty_clouds'] += int(n == 0)
        detail['min_points'] = n if detail['min_points'] is None else min(n, detail['min_points'])
        detail['max_points'] = max(n, detail['max_points'])
        detail['malformed_clouds'] += int(n != msg.point_num)
        if n:
            offsets = [p.offset_time for p in msg.points]
            detail['last_offset_span_ns'] = max(offsets) - min(offsets)
            detail['last_line_ids'] = sorted({p.line for p in msg.points})
            detail['last_tags'] = sorted({p.tag for p in msg.points})
            detail['last_timebase_ns'] = msg.timebase
            sample = msg.points[::max(1, (n + 1023) // 1024)]
            detail['sampled_xyz'] += len(sample)
            detail['nonfinite_xyz'] += sum(not all(math.isfinite(v) for v in (p.x, p.y, p.z))
                                          for p in sample)

    if args.format == 'custom':
        from livox_ros_driver2.msg import CustomMsg
        node.create_subscription(CustomMsg, '/dog_lidar/custom', custom_callback, qos_profile_sensor_data)
    else:
        node.create_subscription(PointCloud2, '/dog_lidar/points', cloud_callback, qos_profile_sensor_data)
    # Decoding a 20k-point Python CustomMsg can occupy the callback thread
    # longer than the default five-message IMU queue (25 ms at 200 Hz).
    # Buffer the audit subscriber; this changes no sensor/control setting.
    imu_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)
    detail['imu_subscriber_queue_depth'] = 512
    node.create_subscription(Imu, '/dog_lidar/imu', imu_callback, imu_qos)
    start = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=min(0.1, args.seconds))
    except KeyboardInterrupt:
        interrupted = True
    finally:
        finish = time.monotonic()
        report = dict(duration_s=finish - start, interrupted=interrupted,
                      streams={key: value.report(finish) for key, value in stats.items()},
                      details=detail,
                      interpretation='Observation only. Silence is not proven packet loss; stamp age requires a common clock. XYZ validity is sampled. No mounting or navigation validation.')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
