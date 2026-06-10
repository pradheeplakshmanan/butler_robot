#!/usr/bin/env python3

# Order Publisher - Milestone 1
# Sends a table order to the butler robot
# Change the table variable to test different tables

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OrderPublisher(Node):

    def __init__(self):
        super().__init__('order_publisher')

        self.publisher = self.create_publisher(String, '/butler/order', 10)

        # Wait 2 seconds then send order
        self.timer = self.create_timer(2.0, self.send_order)
        self.sent = False

        self.get_logger().info('Order Publisher Ready!')

    def send_order(self):
        if self.sent:
            return

        # Change table1 to table2 or table3 to test other tables
        table = 'table1'

        msg = String()
        msg.data = table
        self.publisher.publish(msg)

        self.get_logger().info(f'Order sent for {table}!')
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = OrderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()