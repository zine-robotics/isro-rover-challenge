from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the serial port argument
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',  # Default port
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
        )
    ])

#ros2 launch hardware base_hardware.launch.py serial_port:=/dev/ttyUSB0