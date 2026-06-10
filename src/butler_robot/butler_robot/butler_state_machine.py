#!/usr/bin/env python3

# Butler Robot - Milestone 1
# Simple delivery: Home -> Kitchen -> Table -> Home

import rclpy
import math
import yaml
import os

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ButlerRobot(Node):

    def __init__(self):
        super().__init__('butler_robot')

        # callback group for concurrency safety
        self.cb_group = ReentrantCallbackGroup()

        # Nav2 Action Client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Load waypoints
        self.waypoints = self.load_waypoints()

        # State machine state
        self.state = 'IDLE'

        # Track order
        self.current_table = None

        # Subscriber for orders
        self.order_sub = self.create_subscription(
            String,
            '/butler/order',
            self.order_callback,
            10,
            callback_group=self.cb_group
        )

        # Wait once for Nav2 server (IMPORTANT FIX)
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server is ready!")

        self.get_logger().info('Butler Robot Ready! Waiting for orders...')

    # ---------------- LOAD WAYPOINTS ----------------
    def load_waypoints(self):
        config_path = os.path.join(
            get_package_share_directory('butler_robot'),
            'config',
            'waypoints.yaml'
        )

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)

        return data['waypoints']

    # ---------------- ORDER CALLBACK ----------------
    def order_callback(self, msg):

        if self.state != 'IDLE':
            self.get_logger().warn('Robot busy! Ignoring new order.')
            return

        self.current_table = msg.data
        self.get_logger().info(f'Order received for {self.current_table}!')

        self.state = 'BUSY'

        self.get_logger().info('Starting delivery...')
        self.go_to('kitchen')

    # ---------------- SEND GOAL ----------------
    def go_to(self, location):

        self.get_logger().info(f'Going to {location}...')

        wp = self.waypoints[location]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])

        yaw = float(wp['yaw'])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # send goal (ASYNC ONLY)
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

        self._current_location = location

    # ---------------- GOAL RESPONSE ----------------
    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.state = 'IDLE'
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ---------------- RESULT CALLBACK ----------------
    def result_callback(self, future):

        self.get_logger().info(f"Reached {self._current_location}!")

        location = self._current_location

        # State machine transitions
        if location == 'kitchen':
            self.go_to(self.current_table)

        elif location == self.current_table:
            self.go_to('home')

        elif location == 'home':
            self.get_logger().info("Delivery complete! Back at home.")
            self.state = 'IDLE'
            self.current_table = None


# ---------------- MAIN ----------------
def main(args=None):

    rclpy.init(args=args)

    robot = ButlerRobot()

    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()