#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from datetime import datetime
from std_msgs.msg import String

class PublishTimeNode(Node): 
    def __init__(self):
        super().__init__("pub_time") 

        self.student_name = "panho"
        self.publisher_ = self.create_publisher(String, "current_time", 10)
        self.timer_ = self.create_timer(3, self.publish_callback)
        self.get_logger().info("The Timer has been started!")

    def publish_callback(self):
        current_time = datetime.now()
        current_time_str = current_time.strftime('%Y-%m-%d %H:%M:%S')

        msg = String()
        msg.data = f"\nThis is {self.student_name} clock" + "\n" + f"Current time is {current_time_str}"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PublishTimeNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()