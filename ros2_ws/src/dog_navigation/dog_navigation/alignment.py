"""One-shot saved-map alignment; subsequent motion comes only from LIO."""
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def pose_values(pose):
    p, q = pose.position, pose.orientation
    values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or not .99 < np.linalg.norm(values[3:]) < 1.01:
        raise ValueError('Invalid pose')
    return values[:3], Rotation.from_quat(values[3:])


def map_alignment(map_position, map_rotation, odom_position, odom_rotation):
    """Planar map heading offset; preserve LIO roll/pitch and vertical motion."""
    yaw = map_rotation.as_euler('xyz')[2] - odom_rotation.as_euler('xyz')[2]
    rotation = Rotation.from_euler('z', yaw)
    return np.asarray(map_position) - rotation.apply(odom_position), rotation


class FixedAlignment(Node):
    def __init__(self):
        super().__init__('dog_nav_alignment')
        self.odom = None
        self.offset = None
        self.controller = None
        self.tf = TransformBroadcaster(self)
        self.status = self.create_publisher(String, '/dog_nav/alignment_status', 10)
        self.create_subscription(Odometry, '/dog_nav/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/dog_nav/initialpose', self.align, 10)
        self.create_subscription(String, '/dog_ik_walk_imu/status', self.receive_controller, 1)
        self.create_timer(1., self.publish_status)
        self.reason = 'waiting_for_initial_map_pose'

    def receive_controller(self, msg):
        try:
            self.controller = (time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            self.controller = None

    def align(self, msg):
        try:
            if self.offset is not None:
                raise ValueError('Alignment already locked; restart localization to set a new origin')
            if msg.header.frame_id != 'dog_lio_map':
                raise ValueError('Initial pose must use dog_lio_map')
            if self.odom is None or time.monotonic() - self.odom[0] > .5:
                raise ValueError('Fresh LIO odometry required')
            if self.controller is None or time.monotonic() - self.controller[0] > 1.5:
                raise ValueError('Controller status required before alignment')
            state = self.controller[1]
            if state.get('teleop', {}).get('active') or state.get('mode') not in (None, 'hold', 'stand'):
                raise ValueError('Stop motion before setting the initial pose')
            mp, mr = pose_values(msg.pose.pose)
            op, ore = pose_values(self.odom[1].pose.pose)
            self.offset = map_alignment(mp, mr, op, ore)
            self.reason = 'locked_initial_alignment_lio_tracking'
            self.get_logger().info(self.reason)
            self.receive_odom(self.odom[1])
        except (ValueError, TypeError) as error:
            self.get_logger().warning(str(error))
        self.publish_status()

    def receive_odom(self, msg):
        if msg.header.frame_id != 'dog_odom' or msg.child_frame_id != 'dog_base_link':
            return
        self.odom = (time.monotonic(), msg)
        if self.offset is None:
            return
        translation, rotation = self.offset
        t = TransformStamped()
        # Never refresh TF from a wall-clock timer: stale LIO must remain stale.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'dog_lio_map'
        t.child_frame_id = 'dog_odom'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, translation)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, rotation.as_quat())
        self.tf.sendTransform(t)

    def publish_status(self):
        data = dict(locked=self.offset is not None, reason=self.reason, source='FAST-LIO lidar + internal IMU')
        if self.offset is not None:
            data.update(translation=self.offset[0].tolist(), quaternion_xyzw=self.offset[1].as_quat().tolist())
        self.status.publish(String(data=json.dumps(data)))


def main():
    rclpy.init()
    node = FixedAlignment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
