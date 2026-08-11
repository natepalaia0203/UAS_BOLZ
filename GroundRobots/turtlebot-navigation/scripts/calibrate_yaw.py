import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from marvelmind_ros2_msgs.msg import HedgePosition
import math
import json
import time

CALIB_FILE = '/home/uas/yaw_offset.json'
DRIVE_SPEED = 0.1
DRIVE_DISTANCE = 0.5

class YawCalibrator(Node):
    def __init__(self):
        super().__init__('yaw_calibrator')
        self.x = None
        self.y = None
        self.raw_yaw = 0.0
        self.cmd_pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.create_subscription(HedgePosition, '/hedgehog_pos', self.hedge_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.get_logger().info('Yaw calibrator started. Waiting for position...')

    def hedge_callback(self, msg):
        self.x = msg.x_m
        self.y = msg.y_m

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.raw_yaw = math.atan2(siny_cosp, cosy_cosp)

    def calibrate(self):
        self.get_logger().info('Waiting for Marvelmind position...')
        while self.x is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        x_start = self.x
        y_start = self.y
        self.get_logger().info(f'Start position: ({x_start:.3f}, {y_start:.3f})')
        self.get_logger().info(f'Driving {DRIVE_DISTANCE}m forward for calibration...')

        cmd = Twist()
        cmd.linear.x = DRIVE_SPEED
        start_time = time.time()
        drive_time = DRIVE_DISTANCE / DRIVE_SPEED

        while time.time() - start_time < drive_time:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)

        x_end = self.x
        y_end = self.y
        self.get_logger().info(f'End position: ({x_end:.3f}, {y_end:.3f})')

        dx = x_end - x_start
        dy = y_end - y_start
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.1:
            self.get_logger().error(f'Robot barely moved ({dist:.3f}m)! Check Kobuki is on and moving.')
            return False

        heading_truth = math.atan2(dy, dx)
        yaw_offset = heading_truth - self.raw_yaw

        while yaw_offset > math.pi:
            yaw_offset -= 2 * math.pi
        while yaw_offset < -math.pi:
            yaw_offset += 2 * math.pi

        self.get_logger().info(f'Distance moved: {dist:.3f}m')
        self.get_logger().info(f'True heading: {math.degrees(heading_truth):.1f} degrees')
        self.get_logger().info(f'IMU raw yaw: {math.degrees(self.raw_yaw):.1f} degrees')
        self.get_logger().info(f'YAW OFFSET: {math.degrees(yaw_offset):.1f} degrees')

        data = {'yaw_offset': yaw_offset}
        with open(CALIB_FILE, 'w') as f:
            json.dump(data, f)
        self.get_logger().info(f'Saved to {CALIB_FILE}')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = YawCalibrator()
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.calibrate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
