import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

arm_action_server_dir= get_package_share_directory('arm_action_server')

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
        ),
        Node(
            package='arm_planner', 
            executable='arm_planner_api',  
            output='screen'
        ),
        Node(
            package='action_servers', 
            executable='pickup_object', 
            output='screen'
        ),
        Node(
            package='action_servers', 
            executable='drop_object', 
            output='screen'
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            arm_action_server_dir, 'launch', 'arm_action_server_withgripper.launch.py')
        )),

    ])

