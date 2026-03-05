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
import rclpy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


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
        # add your code here to adjust your movement based on 2D pose feedback
        pass

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