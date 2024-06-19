import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

rover_dir = get_package_share_directory('zine_rover_description')

def generate_launch_description():
    return LaunchDescription([
        # Declare the serial port argument
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',  # Default port
            description='Serial port for connecting to Arduino'
        ),

        # Node for twist_to_arduino
        Node(
            package='hardware',
            executable='twist_to_arduino',
            name='twist_to_arduino',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}]
        ),

        # Node for odom_publisher
        Node(
            package='hardware',
            executable='odom_publisher',
            name='odom_publisher'
        ),
        
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
           rover_dir, 'launch', 'real.launch.launch.py')
        ))
    ])

