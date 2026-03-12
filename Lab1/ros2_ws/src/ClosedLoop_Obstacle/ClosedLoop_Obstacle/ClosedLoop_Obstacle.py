import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# 开环控制：前方有障碍时速度为 0，否则以设定速度前进
FRONT_THRESHOLD = 0.3   # 前方障碍距离阈值 (m)
LINEAR_SPEED = 0.5      # 无障碍时前进线速度 (m/s)
FRONT_IDX = 0           # 激光正前方索引（与 laser 节点一致）


def main(args=None):
    rclpy.init(args=args)
    node = Node('closed_loop_obstacle_node')

    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    def scan_callback(msg: LaserScan):
        # 直接使用 scan：前方距离
        front_range = msg.ranges[FRONT_IDX]
        twist = Twist()
        if front_range < FRONT_THRESHOLD:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = float(LINEAR_SPEED)
            twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        node.get_logger().info('published')

    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.get_logger().info('Open-loop front obstacle avoidance: subscribing to /scan, publishing to cmd_vel')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
