#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Mover(Node):

    def __init__(self):
        super().__init__('vel_publisher')

        # 创建 publisher，发布到 cmd_vel 话题
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # 每 1 秒调用一次 timer_callback
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.i = 0
        self.get_logger().info("Press CTRL + C to terminate")

    def timer_callback(self):
        msg = Twist()

        # 在这里设置速度
        # 示例：机器人前进
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.5

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")


def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
