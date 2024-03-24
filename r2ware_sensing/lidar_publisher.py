import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class PointCloudTransformer(Node):

    def __init__(self):
        super().__init__('pointcloud_transformer')

        # Create publishers
        self.pub_raw = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', QoSProfile(depth=5))
        self.pub_raw_ex = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', QoSProfile(depth=5))

        # Create subscribers
        self.sub_raw = self.create_subscription(
            PointCloud2,
            '/aw_points',
            self.raw_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.sub_raw_ex = self.create_subscription(
            PointCloud2,
            '/aw_points_ex',
            self.raw_ex_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.get_logger().info('PointCloudTransformer node initialized')

    def raw_callback(self, msg):
        self.get_logger().info('Received message on /aw_points topic')
        msg.header.frame_id = 'sensor_kit_base_link'
        self.pub_raw.publish(msg)
        self.get_logger().info('Published message on /sensing/lidar/top/pointcloud_raw')

    def raw_ex_callback(self, msg):
        self.get_logger().info('Received message on /aw_points_ex topic')
        msg.header.frame_id = 'sensor_kit_base_link'
        self.pub_raw_ex.publish(msg)
        self.get_logger().info('Published message on /sensing/lidar/top/pointcloud_raw_ex')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_transformer = PointCloudTransformer()
    rclpy.spin(pointcloud_transformer)
    pointcloud_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
