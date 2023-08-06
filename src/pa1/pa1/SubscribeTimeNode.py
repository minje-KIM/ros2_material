#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SubscribeTimeNode(Node): 
    def __init__(self):
        super().__init__("sub_time") 
        self.subscriber_ = self.create_subscription(
            String, "current_time", self.subscribe_callback, 10)

    def subscribe_callback(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SubscribeTimeNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()