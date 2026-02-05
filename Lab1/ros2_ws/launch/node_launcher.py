#! /usr/bin/python
import launch
import launch_ros.actions
def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='r2',
            executable='prt',
            output='screen',
            name='custom_py_node_name'
        )
    ])