#!/usr/bin/env python3
"""
Cancel Order Publisher
Cancels a specific table order
Usage: ros2 run butler_robot cancel_order
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CancelOrder(Node):

    def __init__(self):
        super().__init__('cancel_order')
        self.publisher = self.create_publisher(
            String, '/butler/cancel_table', 10)
        self.timer = self.create_timer(2.0, self.send_cancel)
        self.sent = False

    def send_cancel(self):
        if self.sent:
            return
        msg = String()
        # Change to whichever table to cancel
        msg.data = 'table2'
        self.publisher.publish(msg)
        self.get_logger().info(f'Cancel sent for {msg.data}!')
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = CancelOrder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()