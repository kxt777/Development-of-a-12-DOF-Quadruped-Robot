"""Publish a saved dog_lio_map point cloud for offline inspection only."""
import argparse
import numpy as np
from .map_preview import read_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True)
    args, ros_args = parser.parse_known_args()
    xyz = read_pcd(args.pcd)
    xyz = np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)], dtype='<f4')
    if not len(xyz):
        raise ValueError('Saved map has no finite XYZ points')
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    rclpy.init(args=ros_args)
    node = rclpy.create_node('dog_saved_map')
    publisher = node.create_publisher(PointCloud2, '/dog_mapping/saved_map',
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg = PointCloud2()
    msg.header.frame_id = 'dog_lio_map'
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(('x', 'y', 'z'))]
    msg.point_step, msg.row_step = 12, 12 * len(xyz)
    msg.is_dense, msg.is_bigendian = True, False
    msg.data = xyz.tobytes()
    # Periodic retransmission supports late viewers; data remain a saved snapshot.
    def publish():
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
    node.create_timer(2.0, publish)
    publish()
    node.get_logger().info('Saved snapshot: %d points from %s' % (len(xyz), args.pcd))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
