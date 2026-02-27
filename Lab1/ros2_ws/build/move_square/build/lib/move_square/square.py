#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class Mover(Node):
    def __init__(self):
        super().__init__('square_mover')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_orig', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 参数
        self.linear_speed = 0.8
        self.angular_speed = 0.5

        self.forward_time = 4.0 / self.linear_speed
        self.turn_time = (math.pi / 2) / self.angular_speed
        self.turn_time = self.turn_time - 0.05
        self.phase = "forward"
        self.time_count = 0.0
        self.edge_count = 0

        self.get_logger().info("Start square movement 1.1")


        # 保存最新位姿
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0

        # 订阅 odom
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Gazebo reset service client（Gazebo Classic 常用）
        from std_srvs.srv import Empty
        self.reset_cli = self.create_client(Empty, '/reset_world')

    def timer_callback(self):


        msg = Twist()
        self.time_count += 0.1

        if self.phase == "forward":

            msg.linear.x = self.linear_speed

            if self.time_count >= self.forward_time:
                self.phase = "turn"
                self.get_logger().info("turn")
                self.time_count = 0.0

        elif self.phase == "turn":

            msg.angular.z = self.angular_speed

            if self.time_count >= self.turn_time:
                self.edge_count += 1
                self.time_count = 0.0

                if self.edge_count >= 4:
                    self.stop_robot()
                    return

                self.phase = "forward"

        self.publisher_.publish(msg)

    def odom_callback(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        self.timer.cancel()
        self.publisher_.publish(Twist())
        self.get_logger().info("Square completed")
        rclpy.shutdown()


    def reset_gazebo(self):
        from std_srvs.srv import Empty

        if not self.reset_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /reset_world not available")
            return False

        req = Empty.Request()
        future = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            self.get_logger().error("Reset service call failed or timed out")
            return False

        self.get_logger().info("Gazebo world reset done")
        return True

def main(args=None):
    rclpy.init(args=args)

    mover = Mover()
    rclpy.spin(mover)   # 会因为 shutdown 退出

    print(f"x={mover.last_x:.3f}, y={mover.last_y:.3f}, yaw={mover.last_yaw:.3f}")

    mover.reset_gazebo()

    mover.destroy_node()

if __name__ == '__main__':
    main()
