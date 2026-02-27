#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import math

class Mover(Node):

    def __init__(self):
        super().__init__('square_mover')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 参数
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        self.forward_time = 4.0 / self.linear_speed
        self.turn_time = (math.pi / 2) / self.angular_speed
        self.turn_time = self.turn_time - 0.05
        self.phase = "forward"
        self.time_count = 0.0
        self.edge_count = 0

        self.get_logger().info("Start square movement")

    def timer_callback(self):

        msg = Twist()
        self.time_count += 0.1

        if self.phase == "forward":

            msg.linear.x = self.linear_speed

            if self.time_count >= self.forward_time:
                self.phase = "turn"

                stop = Twist()
                for _ in range(10):              # 连发几次更稳
                    self.publisher_.publish(stop)

                self.time_count = 0.0

        elif self.phase == "turn":

            msg.angular.z = self.angular_speed

            if self.time_count >= self.turn_time:
                self.edge_count += 1
                stop = Twist()
                for _ in range(10):              # 连发几次更稳
                    self.publisher_.publish(stop)
                self.time_count = 0.0

                if self.edge_count >= 4:
                    self.stop_robot()
                    return

                self.phase = "forward"

        self.publisher_.publish(msg)

    def stop_robot(self):
        # 先停止定时器，防止继续发控制
        self.timer.cancel()

        stop = Twist()
        for _ in range(10):              # 连发几次更稳
            self.publisher_.publish(stop)

        self.get_logger().info("Square completed, stopping and exiting")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    rclpy.spin(mover)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
