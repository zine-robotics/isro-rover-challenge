import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='hardware',
            executable='odom_node',
            output='screen',
            parameters=[{"serial_port":"1"}]
        ),

        Node(
            package='hardware', 
            executable='arduino_twist',  
            output='screen'
        )
    ])
