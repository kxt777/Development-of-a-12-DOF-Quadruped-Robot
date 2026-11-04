"""Bounded, sensor-only FAST-LIO capture with raw rosbag and explicit PCD save.

Run on the NUC with the ROS workspace sourced. The lidar driver must already run.
No motor topic, service, or command is used.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def record_worker(output):
    import rosbag2_py
    options = rosbag2_py.RecordOptions()
    options.topics = ['/dog_lidar/custom', '/dog_lidar/imu', '/dog_mapping/odometry']
    options.node_prefix = 'dog_mapping_'
    options.rmw_serialization_format = 'cdr'
    storage = rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3')
    try:
        rosbag2_py.Recorder().record(storage, options)
    except KeyboardInterrupt:
        pass


def stop_owned(process):
    if process is None:
        return None
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    return process.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--record-worker', action='store_true')
    parser.add_argument('--until-stopped', action='store_true',
                        help='Capture until OUTPUT/stop.request exists, then save normally')
    args = parser.parse_args()
    if args.record_worker:
        record_worker(args.output)
        return
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be positive and finite')
    import fcntl
    lock = open('/tmp/dog_mapping_capture.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if subprocess.run(['pgrep', '-x', 'fastlio_mapping'], capture_output=True).returncode == 0:
        raise RuntimeError('FAST-LIO already running; keep the existing session')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, Imu
    from std_srvs.srv import Trigger
    rclpy.init()
    node = rclpy.create_node('dog_mapping_session')
    report = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'requested_seconds': args.seconds, 'motor_commands_sent': False,
              'until_stopped': args.until_stopped,
              'poses': [], 'cloud_count': 0, 'imu_count': 0,
              'imu_acceleration_units': 'g (raw Livox)',
              'stationary_or_moving': 'not inferred; user must identify physical motion'}
    qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        report['poses'].append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def on_cloud(msg):
        report['cloud_count'] += 1
        report['last_cloud_points'] = msg.width * msg.height

    def on_imu(msg):
        report['imu_count'] += 1

    node.create_subscription(Odometry, '/dog_mapping/odometry', on_odom, qos)
    node.create_subscription(PointCloud2, '/dog_mapping/cloud_registered', on_cloud, 10)
    node.create_subscription(Imu, '/dog_lidar/imu', on_imu, qos)
    client = node.create_client(Trigger, '/dog_mapping/save_map')
    logs = []
    mapper = recorder = None
    try:
        for name in ('mapping', 'recording'):
            logs.append((args.output / (name + '.log')).open('wb'))
        recorder = subprocess.Popen([sys.executable, __file__, '--record-worker',
                                     '--output', str(args.output / 'raw_bag')],
                                    stdout=logs[1], stderr=subprocess.STDOUT, start_new_session=True)
        mapper = subprocess.Popen(['ros2', 'launch', 'dog_navigation', 'dog_mapping.launch.py',
                                   'map_path:=' + str(args.output / 'map.pcd')],
                                  stdout=logs[0], stderr=subprocess.STDOUT, start_new_session=True)
        report['processes'] = {'supervisor': os.getpid(), 'mapper_launch': mapper.pid,
                               'recorder_worker': recorder.pid}
        (args.output / 'started.json').write_text(json.dumps(report, indent=2))
        print('STARTED ' + str(args.output), flush=True)
        start = time.monotonic()
        next_status = start + 5
        while rclpy.ok() and (args.until_stopped or time.monotonic() - start < args.seconds):
            if (args.output / 'stop.request').exists():
                report['stop_reason'] = 'operator_stop_file'
                break
            if mapper.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('Mapping or recording exited early; inspect session logs')
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_status:
                print('PROGRESS clouds=%d odometry=%d imu=%d' % (
                    report['cloud_count'], len(report['poses']), report['imu_count']), flush=True)
                live = {k: v for k, v in report.items() if k != 'poses'}
                live.update(status='recording', elapsed_seconds=time.monotonic()-start,
                            odometry_count=len(report['poses']),
                            last_pose=report['poses'][-1] if report['poses'] else None,
                            updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
                temporary = args.output / 'live.json.tmp'
                temporary.write_text(json.dumps(live, indent=2))
                temporary.replace(args.output / 'live.json')
                next_status += 5
        report['observed_seconds'] = time.monotonic() - start
        if not report['poses'] or not report['cloud_count']:
            raise RuntimeError('No mapped cloud or odometry received')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Map save service unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Map save failed or timed out')
        pcd = args.output / 'map.pcd'
        payload = pcd.read_bytes()
        header, sep, _ = payload.partition(b'DATA binary\n')
        if not sep:
            raise RuntimeError('Saved map missing expected binary PCD header')
        fields = dict(line.split(' ', 1) for line in header.decode('ascii').splitlines()
                      if ' ' in line and not line.startswith('#'))
        count = int(fields['POINTS'])
        if count <= 0:
            raise RuntimeError('Saved map contains no points')
        report['map'] = {'points': count, 'bytes': len(payload),
                         'sha256': hashlib.sha256(payload).hexdigest()}
        report['status'] = 'saved'
    except BaseException as exc:
        report['status'] = 'incomplete'
        report['error'] = repr(exc)
        raise
    finally:
        report['mapper_launch_exit'] = stop_owned(mapper)
        report['recorder_worker_exit'] = stop_owned(recorder)
        report['ended_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        report['rosbag_metadata_present'] = (args.output / 'raw_bag/metadata.yaml').exists()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        (args.output / 'live.json').write_text(json.dumps(
            {k: v for k, v in report.items() if k != 'poses'}, indent=2), encoding='utf-8')
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('FINISHED ' + report['status'], flush=True)


if __name__ == '__main__':
    main()
