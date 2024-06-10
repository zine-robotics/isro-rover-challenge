import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_joint_trajectory_server',
            executable='trajectory_action_server',
            output='screen',
        ),
        # Node(
        #     package='follow_joint_trajectory_server',
        #     executable='arm_serial_arduino_node',
        #     output='screen',
        # ),

        Node(
            package='follow_joint_trajectory_server',  # Replace with your package name
            executable='joint_state_publisher',  # Replace with your script name
            # name='joint_state_publisher',
            output='screen'
        )
    ])
