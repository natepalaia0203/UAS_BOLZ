import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from marvelmind_ros2_msgs.msg import HedgePosition
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import json
import os

CALIB_FILE = '/home/uas/yaw_offset.json'

def load_yaw_offset():
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE, 'r') as f:
            data = json.load(f)
        offset = data.get('yaw_offset', 0.0)
        print(f'Loaded yaw offset: {math.degrees(offset):.1f} degrees')
        return offset
    print('No calibration file found, using 0.0 offset')
    return 0.0

class FusionNode(Node):
    def __init__(self):
        super().__init__('marvelmind_fusion')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_imu = False
        self.yaw_offset = load_yaw_offset()
        self.odom_pub = self.create_publisher(Odometry, '/marvelmind_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(HedgePosition, '/hedgehog_pos', self.hedge_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.get_logger().info(
            f'Fusion Node started! Yaw source: MicroStrain /imu/data, '
            f'offset: {math.degrees(self.yaw_offset):.1f} deg')

        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'map'
        static_t.child_frame_id = 'odom'
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.0
        static_t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_t)

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = raw_yaw + self.yaw_offset
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        self.have_imu = True

    def hedge_callback(self, msg):
        if not self.have_imu:
            self.get_logger().warn('No IMU data yet, skipping odom publish', throttle_duration_sec=2.0)
            return

        self.x = msg.x_m
        self.y = msg.y_m
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
