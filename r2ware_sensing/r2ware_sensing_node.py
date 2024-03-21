import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, Imu
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class TopicForwarder(Node):
    def __init__(self):
        super().__init__('r2ware_sensing_node')

        # Subscriber for clock topic
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # Camera topics
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.image_raw_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_raw_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10)

        # IMU topic
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 30)

        # Camera forward topics
        self.camera_info_pub = self.create_publisher(CameraInfo, '/sensing/camera/traffic_light/camera_info', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=1))
        self.image_raw_pub = self.create_publisher(Image, '/sensing/camera/traffic_light/image_raw', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=1))

        # IMU forward topic
        self.imu_pub = self.create_publisher(Imu, '/sensing/imu/tamagawa/imu_raw', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=1000))

        # GNSS forward topics
        self.gnss_pose_pub = self.create_publisher(Pose, '/sensing/gnss/pose', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=1))
        self.gnss_pose_covariance_pub = self.create_publisher(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.RELIABLE,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=1))

        # Top LiDAR forward topics
        self.lidar_top_raw_pub = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=5))
        self.lidar_top_raw_ex_pub = self.create_publisher(PointCloud2, '/sensing/lidar/top/pointcloud_raw_ex', QoSProfile(
                                                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                        history=QoSHistoryPolicy.KEEP_LAST,
                                                        depth=5))

        self.latest_clock_msg = None

    def clock_callback(self, msg):
        self.latest_clock_msg = msg

    def camera_info_callback(self, msg):
        if self.latest_clock_msg is not None:
            msg.header.stamp = self.latest_clock_msg.clock
            msg.header.frame_id = "traffic_light_left_camera/camera_optical_link"
            self.camera_info_pub.publish(msg)

    def image_raw_callback(self, msg):
        if self.latest_clock_msg is not None:
            msg.header.stamp = self.latest_clock_msg.clock
            msg.header.frame_id = "traffic_light_left_camera/camera_optical_link"
            self.image_raw_pub.publish(msg)

    def pointcloud_callback(self, msg):
        if self.latest_clock_msg is not None:
            msg.header.stamp = self.latest_clock_msg.clock
            msg.header.frame_id = "sensor_kit_base_link"
            self.lidar_top_raw_pub.publish(msg)
            self.lidar_top_raw_ex_pub.publish(msg)
            

    def imu_callback(self, msg):
        if self.latest_clock_msg is not None:
            msg.header.stamp = self.latest_clock_msg.clock
            msg.header.frame_id = "tamagawa/imu_link"
            self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    forwarder = TopicForwarder()

    rclpy.spin(forwarder)

    forwarder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
