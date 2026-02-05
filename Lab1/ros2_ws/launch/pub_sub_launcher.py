#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publisher = Node(
        package='r2',
        executable='pub',
        name='topic_publisher',
        output='screen',
        parameters=[],
    )

    subscriber = Node(
        package='r2',
        executable='sub',
        name='topic_subscriber',
        output='screen',
        parameters=[],
    )

    return LaunchDescription([
        publisher,
        subscriber,
    ])
