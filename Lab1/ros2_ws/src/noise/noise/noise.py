import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AddMotionNoise(Node):
    def __init__(self):
        super().__init__('add_motion_noise')

        # Parameters
        self.declare_parameter('linear_noise', 0.3)     # std dev for linear.x
        self.declare_parameter('angular_noise', 0.3)    # std dev for angular.z
        self.declare_parameter('sub_topic_name', 'cmd_vel_orig')
        self.declare_parameter('pub_topic_name', 'cmd_vel')


        self.sub_topic_name = self.get_parameter('sub_topic_name').value
        self.pub_topic_name = self.get_parameter('pub_topic_name').value

        self.get_logger().info(f"subscriber topic_name: {self.sub_topic_name}")
        self.get_logger().info(f"publisher topic_name: {self.pub_topic_name}")

        self.publisher_ = self.create_publisher(Twist, self.pub_topic_name, 10)
        self.subscription_ = self.create_subscription(
            Twist,
            self.sub_topic_name,
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('AddMotionNoise has been initialized.')

    def cmd_vel_callback(self, msg: Twist):
        # Read parameters (so you can change them with ros2 param set while running)
        linear_std = float(self.get_parameter('linear_noise').value)
        angular_std = float(self.get_parameter('angular_noise').value)

        # Sample independent Gaussian noises
        n_lin = np.random.normal(0.0, linear_std)
        n_ang = np.random.normal(0.0, angular_std)

        # Create output message
        out = Twist()
        out.linear.x = msg.linear.x + n_lin
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z

        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z + n_ang


        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AddMotionNoise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()