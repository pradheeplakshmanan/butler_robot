#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class CancelOrder(Node):
    def __init__(self):
        super().__init__('cancel_order')
        self.publisher = self.create_publisher(Bool, '/butler/cancel', 10)
        self.timer = self.create_timer(2.0, self.send_cancel)
        self.sent = False

    def send_cancel(self):
        if self.sent:
            return
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info('Cancel signal sent!')
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = CancelOrder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()