"""Measured mounting transform converts FAST-LIO IMU pose to the dog's base."""
import json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster


def base_pose(position,quaternion,t_base_imu,q_base_imu):
    ri=Rotation.from_quat(quaternion);rbi=Rotation.from_quat(q_base_imu)
    rb=ri*rbi.inv()
    return np.asarray(position)-rb.apply(t_base_imu),rb


class OdometryAdapter(Node):
    def __init__(self):
        super().__init__('dog_nav_odometry')
        self.declare_parameter('mounting_config','')
        mount=json.loads(Path(self.get_parameter('mounting_config').value).read_text())
        if not (mount.get('calibrated') or (mount.get('orientation_confirmed') and mount.get('translation_assumption_accepted'))):
            raise ValueError('Mounting convention is not configured')
        self.translation=np.asarray(mount['base_from_lidar_imu_translation_m'],dtype=float)
        self.quaternion=np.asarray(mount['base_from_lidar_imu_quaternion_xyzw'],dtype=float)
        if self.translation.shape!=(3,) or self.quaternion.shape!=(4,) or not np.isfinite(np.r_[self.translation,self.quaternion]).all():
            raise ValueError('Invalid mounting transform')
        if not .99<np.linalg.norm(self.quaternion)<1.01:raise ValueError('Mount quaternion is not normalized')
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        transform=TransformStamped();transform.header.stamp=self.get_clock().now().to_msg()
        transform.header.frame_id='dog_base_link';transform.child_frame_id='dog_lidar_imu'
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,self.translation)
        transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=map(float,self.quaternion)
        self.static.sendTransform(transform)
        self.pub=self.create_publisher(Odometry,'/dog_nav/odom',10)
        self.health=self.create_publisher(String,'/dog_nav/odometry_status',1)
        self.previous=None;self.fault=None
        self.create_subscription(Odometry,'/dog_mapping/odometry',self.receive,qos_profile_sensor_data)

    def receive(self,msg):
        if self.fault:return
        try:
            if msg.header.frame_id!='dog_odom' or msg.child_frame_id!='dog_lidar_imu':raise ValueError('FAST-LIO navigation frame configuration mismatch')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            values=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
            if not np.isfinite(values).all() or not .99<np.linalg.norm(values[3:])<1.01:raise ValueError('Invalid LIO pose')
            pos,rot=base_pose(values[:3],values[3:],self.translation,self.quaternion)
            velocity=np.zeros(3);omega=np.zeros(3)
            if self.previous:
                old_stamp,old_pos,old_rot=self.previous;dt=stamp-old_stamp
                if not 0<dt<=.5:raise ValueError('LIO stream interrupted/reset; restart localization and set map pose before rearming')
                velocity=rot.inv().apply((pos-old_pos)/dt)
                omega=(old_rot.inv()*rot).as_rotvec()/dt
            self.previous=(stamp,pos,rot)
            out=Odometry();out.header=msg.header;out.child_frame_id='dog_base_link'
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pos)
            quat=rot.as_quat()
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
            # Pose covariance is left unspecified: raw LIO error-state covariance
            # cannot be relabeled as base-frame covariance without its convention.
            # Nav2 consumes pose and twist; no second estimator fuses this message.
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,velocity)
            out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,omega)
            transform=TransformStamped();transform.header=out.header;transform.child_frame_id=out.child_frame_id
            transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,pos)
            transform.transform.rotation=out.pose.pose.orientation
            self.tf.sendTransform(transform);self.pub.publish(out)
            self.health.publish(String(data=json.dumps({'ok':True,'source':'FAST-LIO lidar + internal IMU','stamp':stamp})))
        except (ValueError,TypeError) as error:
            self.fault=str(error);self.get_logger().error(self.fault)
            self.health.publish(String(data=json.dumps({'ok':False,'fault':self.fault})))


def main():
    rclpy.init();node=OdometryAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
