#!/usr/bin/env python3
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

"""
Run the turtlebot3 square motion 10 times, resetting Gazebo before each run.
Saves only each trial's final stopping position (x, y) to a single file: final_positions.csv.
"""

import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion


NUM_TRIALS = 10
SETTLE_AFTER_RESET = 2.0  # seconds to wait after reset before starting bot
FINAL_POSITIONS_FILENAME = "final_positions.csv"


def get_current_pose(node, timeout_sec=3.0):
    """Subscribe to /odom, wait for one message, return (x, y) or None."""
    pose = [None]

    def cb(msg):
        if pose[0] is not None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        (_, _, yaw) = euler_from_quaternion(quat)
        pose[0] = (x, y, yaw)

    sub = node.create_subscription(Odometry, "odom", cb, 10)
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and pose[0] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return pose[0]


def main(args=None):
    rclpy.init(args=args)
    node = Node("run_10_trials")
    logger = node.get_logger()

    reset_cli = node.create_client(Empty, "/reset_world")

    if not reset_cli.wait_for_service(timeout_sec=5.0):
        logger.error("Service /reset_world not find")
        node.destroy_node()
        rclpy.shutdown()
        return

    cwd = os.getcwd()
    final_positions = []  # list of (trial_id, x, y) after each run

    for i in range(0, NUM_TRIALS):
        logger.info(f"=== Trial {i}/{NUM_TRIALS} ===")

        # Reset Gazebo world
        req = Empty.Request()
        future = reset_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result() is None:
            logger.error("Reset service call failed or timed out")
            node.destroy_node()
            rclpy.shutdown()
            return
        logger.info("Gazebo reset done, waiting %.1fs ..." % SETTLE_AFTER_RESET)
        time.sleep(SETTLE_AFTER_RESET)

        # Run one lap (bot3 writes trajectory.csv in cwd)
        ret = subprocess.run(
            ["ros2", "run", "turtlebot3", "bot3"],
            cwd=cwd,
            env=os.environ.copy(),
        )
        if ret.returncode != 0:
            logger.warn("bot3 exited with code %d" % ret.returncode)

        # Only record bot's final stopping position (one sample per trial)
        p = get_current_pose(node)
        if p is not None:
            final_positions.append((i, p[0], p[1]))
            logger.info("Trial %d final position: (%.4f, %.4f)" % (i, p[0], p[1]))
        else:
            logger.warn("Could not get pose after trial %d" % i)

    # Write all data to a single file (trial, x, y only)
    out_path = os.path.join(cwd, FINAL_POSITIONS_FILENAME)
    with open(out_path, "w") as f:
        f.write("trial,x,y\n")
        for trial_id, x, y in final_positions:
            f.write("%d,%.6f,%.6f\n" % (trial_id, x, y))
    logger.info("Wrote %d final positions to %s" % (len(final_positions), out_path))

    logger.info("All %d trials finished." % NUM_TRIALS)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
