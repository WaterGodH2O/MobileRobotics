#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('topic_subscriber')

        self.subscription = self.create_subscription(
            String,                # 消息类型
            'phrases',              # 话题名
            self.listener_callback, # 回调函数
            10                      # QoS 队列深度
        )
        self.subscription  # 防止“未使用变量”的警告

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)

    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
