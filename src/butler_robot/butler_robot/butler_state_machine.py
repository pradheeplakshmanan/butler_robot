#!/usr/bin/env python3
# ============================================================
# Butler Robot - Milestone 3
#
# Scenario 3a: Kitchen Timeout -> Home directly
# Scenario 3b: Table Timeout -> Kitchen -> Home
#
# States:
# IDLE
# GOING_TO_KITCHEN
# WAITING_KITCHEN
# GOING_TO_TABLE
# WAITING_TABLE
# RETURNING_TO_KITCHEN
# GOING_HOME
# ============================================================

import rclpy
import os
import math
import yaml
import time
import threading

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory


class ButlerRobot(Node):

    def __init__(self):
        super().__init__('butler_robot')

        # Callback group for concurrency safety
        self.cb_group = ReentrantCallbackGroup()

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Load waypoint locations from YAML
        self.waypoints = self.load_waypoints()

        # Robot state variables
        self.state = 'IDLE'
        self.current_table = None
        self.destination = None

        # Confirmation timeout (seconds)
        self.timeout = 10.0

        # Confirmation flags
        self.kitchen_confirmed = False
        self.table_confirmed = False

        # Order subscriber
        self.create_subscription(
            String,
            '/butler/order',
            self.order_callback,
            10,
            callback_group=self.cb_group
        )

        # Kitchen confirmation subscriber
        self.create_subscription(
            Bool,
            '/butler/kitchen_confirm',
            self.kitchen_confirm_callback,
            10
        )

        # Table confirmation subscriber
        self.create_subscription(
            Bool,
            '/butler/table_confirm',
            self.table_confirm_callback,
            10
        )

        # Wait for Nav2 server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')

        self.get_logger().info('Butler Robot Ready! Waiting for orders...')

    # ---------------- LOAD WAYPOINTS ----------------
    def load_waypoints(self):
        """Load waypoint coordinates from YAML file."""
        config_path = os.path.join(
            get_package_share_directory('butler_robot'),
            'config',
            'waypoints.yaml'
        )
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        return data['waypoints']

    # ---------------- CONFIRMATION CALLBACKS ----------------
    def kitchen_confirm_callback(self, msg):
        """Receive kitchen confirmation."""
        if msg.data:
            self.kitchen_confirmed = True
            self.get_logger().info('Kitchen Confirmed')

    def table_confirm_callback(self, msg):
        """Receive table confirmation."""
        if msg.data:
            self.table_confirmed = True
            self.get_logger().info('Table Confirmed')

    # ---------------- ORDER CALLBACK ----------------
    def order_callback(self, msg):
        """
        New order received.

        Flow:
        IDLE -> GOING_TO_KITCHEN
        """
        if self.state != 'IDLE':
            self.get_logger().warn('Robot Busy!')
            return

        self.current_table = msg.data

        # Reset confirmation flags
        self.kitchen_confirmed = False
        self.table_confirmed = False

        self.get_logger().info(f'Order received for {self.current_table}')

        self.state = 'GOING_TO_KITCHEN'
        self.navigate('kitchen')

    # ---------------- NAVIGATE ----------------
    def navigate(self, location):
        """Send Nav2 goal to requested waypoint."""
        self.destination = location
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

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # ---------------- GOAL RESPONSE ----------------
    def goal_response_callback(self, future):
        """Check whether Nav2 accepted the goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected by Nav2!')
            self.state = 'IDLE'
            return

        self.get_logger().info('Goal Accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ---------------- RESULT CALLBACK ----------------
    def result_callback(self, future):
        """
        Called when navigation finishes.

        Decides next state transition.
        """
        result = future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Failed to reach {self.destination}')
            self.state = 'IDLE'
            return

        self.get_logger().info(f'Reached {self.destination}')

        # ---- State machine transitions ----

        # Kitchen reached -> wait for kitchen confirmation
        if self.state == 'GOING_TO_KITCHEN':
            self.state = 'WAITING_KITCHEN'
            threading.Thread(
                target=self.wait_kitchen,
                daemon=True
            ).start()

        # Table reached -> wait for table confirmation
        elif self.state == 'GOING_TO_TABLE':
            self.state = 'WAITING_TABLE'
            threading.Thread(
                target=self.wait_table,
                daemon=True
            ).start()

        # Returning to kitchen after table timeout -> go home
        elif self.state == 'RETURNING_TO_KITCHEN':
            self.get_logger().info('Back at kitchen. Now going home...')
            self.state = 'GOING_HOME'
            self.navigate('home')

        # Home reached -> delivery complete
        elif self.state == 'GOING_HOME':
            self.get_logger().info('Delivery Complete')
            self.state = 'IDLE'
            self.current_table = None
            self.destination = None

    # ---------------- WAIT KITCHEN ----------------
    def wait_kitchen(self):
        """
        Wait for kitchen confirmation.

        Scenario 3a:
            Kitchen Timeout -> Home directly

        Success:
            Kitchen Confirmed -> Table
        """
        self.get_logger().info('Waiting for kitchen confirmation...')

        start = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9

            # Kitchen confirmed -> go to table
            if self.kitchen_confirmed:
                self.kitchen_confirmed = False
                self.state = 'GOING_TO_TABLE'
                self.navigate(self.current_table)
                return

            # Scenario 3a: Kitchen timeout -> go home directly
            if now - start >= self.timeout:
                self.get_logger().warn('Kitchen Timeout! Going home directly...')
                self.state = 'GOING_HOME'
                self.navigate('home')
                return

            time.sleep(0.1)

    # ---------------- WAIT TABLE ----------------
    def wait_table(self):
        """
        Wait for table confirmation.

        Scenario 3b:
            Table Timeout -> Kitchen first -> Home

        Success:
            Table Confirmed -> Home directly
        """
        self.get_logger().info('Waiting for table confirmation...')

        start = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9

            # Table confirmed -> go home directly
            if self.table_confirmed:
                self.table_confirmed = False
                self.state = 'GOING_HOME'
                self.navigate('home')
                return

            # Scenario 3b: Table timeout -> go kitchen first then home
            if now - start >= self.timeout:
                self.get_logger().warn(
                    'Table Timeout! Going to kitchen first then home...'
                )
                self.state = 'RETURNING_TO_KITCHEN'
                self.navigate('kitchen')
                return

            time.sleep(0.1)


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