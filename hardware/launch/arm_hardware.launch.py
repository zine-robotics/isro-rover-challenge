import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='hardware',
            executable='arm_serial_node',
            output='screen',
            parameters=[{"serial_port":"1"}]
        ),

        Node(
            package='hardware', 
            executable='joint_state_publisher',  
            output='screen'
        )
    ])
