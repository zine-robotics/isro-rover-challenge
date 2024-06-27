import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_action_server',
            executable='adding_gripper',
            output='screen',
        )
    ])
