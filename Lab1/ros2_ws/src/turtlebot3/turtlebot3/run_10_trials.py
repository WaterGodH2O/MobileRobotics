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
Saves each trajectory as trajectory_1.csv, trajectory_2.csv, ... trajectory_10.csv.
"""

import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


NUM_TRIALS = 10
SETTLE_AFTER_RESET = 2.0  # seconds to wait after reset before starting bot


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

    for i in range(1, NUM_TRIALS + 1):
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

        # Rename trajectory.csv to trajectory_N.csv
        src = os.path.join(cwd, "trajectory.csv")
        dst = os.path.join(cwd, "trajectory_%d.csv" % i)
        if os.path.isfile(src):
            if os.path.isfile(dst):
                os.remove(dst)
            os.rename(src, dst)
            logger.info("Saved %s" % os.path.basename(dst))
        else:
            logger.warn("No trajectory.csv produced for trial %d" % i)

    logger.info("All %d trials finished." % NUM_TRIALS)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
