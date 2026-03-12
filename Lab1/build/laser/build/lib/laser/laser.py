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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserNode(Node):
    """Subscribe to /scan and print front, left, back, right distances every 0.5s."""

    # indices: front=0, left=89, back=179, right=269
    FRONT_IDX = 0
    LEFT_IDX = 89
    BACK_IDX = 179
    RIGHT_IDX = 269

    def __init__(self):
        super().__init__('laser_node')
        self.declare_parameter('scan_topic', 'scan')

        self._latest_scan = None
        self.subscription_ = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        self._print_timer = self.create_timer(0.5, self.print_distances)
        self.get_logger().info(f'Subscribing to /scan')

    def scan_callback(self, msg):
        """Store latest laser scan for periodic printing."""
        self._latest_scan = msg


    def print_distances(self):
        """Print front, left, back, right distances every 0.5s."""

        # self.get_logger().info('Subscribing to /scan')
        # self.get_logger().info(self._latest_scan)
        r = self._latest_scan.ranges
        self.get_logger().info(
            f'Front: {r[self.FRONT_IDX]}, Left: {r[self.LEFT_IDX]}, '
            f'Back: {r[self.BACK_IDX]}, Right: {r[self.RIGHT_IDX]}'
        )



def main(args=None):
    rclpy.init(args=args)
    node = LaserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
