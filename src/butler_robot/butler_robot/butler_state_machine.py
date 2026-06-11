#!/usr/bin/env python3
# ============================================================
# Butler Robot - Milestone 6
#
# Multiple Orders - Skip Unconfirmed Table
# Flow: Home -> Kitchen -> Tables -> Kitchen -> Home
#
# Key difference from M5:
#   After last table -> Kitchen first -> then Home
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

        self.cb_group = ReentrantCallbackGroup()

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.waypoints = self.load_waypoints()

        self.state = 'IDLE'
        self.destination = None
        self.current_goal_handle = None

        self.order_queue = []
        self.current_table = None

        self.timeout = 10.0

        self.kitchen_confirmed = False
        self.table_confirmed = False
        self.cancelled = False

        self.create_subscription(
            String,
            '/butler/order',
            self.order_callback,
            10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            Bool,
            '/butler/kitchen_confirm',
            self.kitchen_confirm_callback,
            10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            Bool,
            '/butler/table_confirm',
            self.table_confirm_callback,
            10,
            callback_group=self.cb_group
        )

        self.create_subscription(
            Bool,
            '/butler/cancel',
            self.cancel_callback,
            10,
            callback_group=self.cb_group
        )

        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server is ready!')
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

    # ---------------- CONFIRMATION CALLBACKS ----------------
    def kitchen_confirm_callback(self, msg):
        if msg.data:
            self.kitchen_confirmed = True
            self.get_logger().info('Kitchen Confirmed')

    def table_confirm_callback(self, msg):
        if msg.data:
            self.table_confirmed = True
            self.get_logger().info('Table Confirmed')

    # ---------------- CANCEL CALLBACK ----------------
    def cancel_callback(self, msg):
        if not msg.data:
            return
        if self.state == 'IDLE':
            self.get_logger().warn('No active order to cancel!')
            return
        self.get_logger().warn(f'Cancel received! Current state: {self.state}')
        self.cancelled = True
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()

    # ---------------- ORDER CALLBACK ----------------
    def order_callback(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn('Robot Busy!')
            return

        # Support multiple tables: "table1,table2,table3"
        tables = [t.strip() for t in msg.data.split(',')]
        self.order_queue = tables

        self.kitchen_confirmed = False
        self.table_confirmed = False
        self.cancelled = False

        self.get_logger().info(f'Orders received for: {self.order_queue}')

        self.state = 'GOING_TO_KITCHEN'
        self.navigate('kitchen')

    # ---------------- NAVIGATE ----------------
    def navigate(self, location):
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

        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # ---------------- GOAL RESPONSE ----------------
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected by Nav2!')
            self.state = 'IDLE'
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal Accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # ---------------- RESULT CALLBACK ----------------
    def result_callback(self, future):
        self.current_goal_handle = None

        result = future.result()
        status = result.status

        # Cancellation handling
        if self.cancelled:
            self.get_logger().warn('Order cancelled! Going home...')
            self.cancelled = False
            self.order_queue.clear()
            self.state = 'GOING_HOME'
            self.navigate('home')
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Failed to reach {self.destination}')
            self.state = 'IDLE'
            return

        self.get_logger().info(f'Reached {self.destination}')

        # ---- State machine transitions ----

        # Kitchen reached -> wait for confirmation
        if self.state == 'GOING_TO_KITCHEN':
            self.state = 'WAITING_KITCHEN'
            threading.Thread(
                target=self.wait_kitchen,
                daemon=True
            ).start()

        # Table reached -> wait for confirmation
        elif self.state == 'GOING_TO_TABLE':
            self.state = 'WAITING_TABLE'
            threading.Thread(
                target=self.wait_table,
                daemon=True
            ).start()

        # M6 KEY CHANGE: Kitchen reached after all tables -> now go home
        elif self.state == 'RETURNING_TO_KITCHEN':
            self.get_logger().info('Back at kitchen. Now going home...')
            self.state = 'GOING_HOME'
            self.navigate('home')

        # Home reached -> all done
        elif self.state == 'GOING_HOME':
            self.get_logger().info('All Deliveries Complete!')
            self.state = 'IDLE'
            self.current_table = None
            self.destination = None
            self.order_queue.clear()

    # ---------------- WAIT KITCHEN ----------------
    def wait_kitchen(self):
        """
        Wait for kitchen confirmation.

        Timeout:   Go home directly
        Confirmed: Start delivering to tables
        """
        self.get_logger().info('Waiting for kitchen confirmation...')

        start = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9

            if self.cancelled:
                self.get_logger().warn('Cancelled at kitchen! Going home...')
                self.cancelled = False
                self.order_queue.clear()
                self.state = 'GOING_HOME'
                self.navigate('home')
                return

            if self.kitchen_confirmed:
                self.kitchen_confirmed = False
                self.get_logger().info(f'Orders to deliver: {self.order_queue}')
                self._go_to_next_table()
                return

            if now - start >= self.timeout:
                self.get_logger().warn('Kitchen Timeout! Going home directly...')
                self.order_queue.clear()
                self.state = 'GOING_HOME'
                self.navigate('home')
                return

            time.sleep(0.1)

    # ---------------- WAIT TABLE ----------------
    def wait_table(self):
        """
        Wait for table confirmation.

        No confirmation -> skip table -> next table
        All tables done -> Kitchen -> Home
        """
        self.get_logger().info(
            f'Waiting for confirmation at {self.current_table}...'
        )

        start = self.get_clock().now().nanoseconds / 1e9

        while rclpy.ok():
            now = self.get_clock().now().nanoseconds / 1e9

            if self.cancelled:
                self.get_logger().warn('Cancelled at table! Going home...')
                self.cancelled = False
                self.order_queue.clear()
                self.state = 'GOING_HOME'
                self.navigate('home')
                return

            # Table confirmed -> next table or kitchen
            if self.table_confirmed:
                self.table_confirmed = False
                self.get_logger().info(f'Delivered to {self.current_table}!')
                self._go_to_next_table()
                return

            # Timeout -> skip table -> next table or kitchen
            if now - start >= self.timeout:
                self.get_logger().warn(
                    f'No confirmation at {self.current_table}! Skipping...'
                )
                self._go_to_next_table()
                return

            time.sleep(0.1)

    # ---------------- NEXT TABLE ----------------
    def _go_to_next_table(self):
        """
        Go to next table in queue.

        M6 KEY CHANGE:
        If queue empty -> Kitchen first -> then Home
        """
        if self.order_queue:
            self.current_table = self.order_queue.pop(0)
            self.get_logger().info(
                f'Delivering to {self.current_table}... '
                f'Remaining: {self.order_queue}'
            )
            self.state = 'GOING_TO_TABLE'
            self.navigate(self.current_table)
        else:
            # M6: Go to kitchen before going home
            self.get_logger().info(
                'All tables done! Going to kitchen before home...'
            )
            self.state = 'RETURNING_TO_KITCHEN'
            self.navigate('kitchen')


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