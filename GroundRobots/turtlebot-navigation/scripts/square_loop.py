import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

class SquareLoop(Node):
    def __init__(self):
        super().__init__('square_loop')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Small 1m x 1m square centered at (2.5, -1.3)
        self.waypoints = [
            (2.0, -0.8),
            (3.0, -0.8),
            (3.0, -1.8),
            (2.0, -1.8),
        ]
        self.current = 0
        self.get_logger().info('Square loop started!')
        self._client.wait_for_server()
        self.send_next_goal()

    def send_next_goal(self):
        x, y = self.waypoints[self.current % len(self.waypoints)]
        self.get_logger().info(f'Waypoint {(self.current % len(self.waypoints)) + 1}/4: ({x}, {y})')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self._send_goal_future = self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted!')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Waypoint reached!')
        else:
            self.get_logger().warn(f'Status: {status}, continuing...')
        self.current += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = SquareLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
