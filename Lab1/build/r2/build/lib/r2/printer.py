#! /usr/bin/python
import rclpy
from rclpy.node import Node
rclpy.init(args=None)
node = Node('printer_node') # Note that there should have no space in node name
node.get_logger().info('HELLO WORLD - ROS2')
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()