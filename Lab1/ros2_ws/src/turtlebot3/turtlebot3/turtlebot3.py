# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python
from math import pi, sqrt, atan2, cos, sin
import numpy as np
import threading
import time
import rclpy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

from controller.controller import Controller


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


class Turtlebot3():

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("turtlebot3_move_square")
        self.node.get_logger().info("Press Ctrl + C to terminate")

        self.vel_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.rate = self.node.create_rate(10)

        t = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        t.start()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()

        self.odom_sub = self.node.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # PD controller for orientation theta only (setpoint updated per waypoint)
        self.theta_controller = Controller(P=1.0, D=0.1, set_point=0.0)

        # Waypoints (e.g. square: forward, left, back, home)
        self.waypoints = [(4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]
        self.angle_threshold = 0.05   # rad, consider aligned
        self.dist_threshold = 0.08    # m, consider reached
        self.linear_speed = 0.15      # m/s when moving forward

        try:
            self.run()
        except KeyboardInterrupt:
            print("Interrupted")
        finally:
            # save trajectory to csv file
            np.savetxt("trajectory.csv", np.array(self.trajectory), delimiter=",")
            self.node.destroy_node()
            rclpy.shutdown()

    def run(self):

        time.sleep(1.0)
        for _ in range(20):
            if self.logging_counter > 0:
                break
            self.rate.sleep()

        for x_star, y_star in self.waypoints:
            # Step 1 & 2: Desired orientation = direction from (x,y) to (x*,y*)
            dx = x_star - self.pose.x
            dy = y_star - self.pose.y
            theta_star = atan2(dy, dx)
            self.theta_controller.setPoint(theta_star)

            # Step 3: Turn until aligned with theta*
            while rclpy.ok():
                err = normalize_angle(theta_star - self.pose.theta)
                if abs(err) < self.angle_threshold:
                    break
                # Pass value so that controller sees normalized error
                current_for_pd = theta_star - err
                omega = self.theta_controller.update(current_for_pd)
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = max(-0.5, min(0.5, omega))
                self.vel_pub.publish(msg)
                self.rate.sleep()

            # Step 4: Move forward, keep adjusting angle and check distance
            while rclpy.ok():
                dx = x_star - self.pose.x
                dy = y_star - self.pose.y
                dist = sqrt(dx * dx + dy * dy)
                if dist < self.dist_threshold:
                    break
                theta_star = atan2(dy, dx)
                self.theta_controller.setPoint(theta_star)
                err = normalize_angle(theta_star - self.pose.theta)
                current_for_pd = theta_star - err
                omega = self.theta_controller.update(current_for_pd)
                msg = Twist()
                msg.linear.x = self.linear_speed
                msg.angular.z = max(-0.5, min(0.5, omega))
                self.vel_pub.publish(msg)
                self.rate.sleep()

            # Step 5: Stop before next waypoint
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            self.rate.sleep()

        self.node.get_logger().info("All waypoints reached.")

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1

        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory

            self.node.get_logger().info(
                "odom: x="
                + str(self.pose.x)
                + "; y="
                + str(self.pose.y)
                + "; theta="
                + str(yaw)
            )


def main(args=None):
    turtlebot = Turtlebot3()


if __name__ == "__main__":
    main()