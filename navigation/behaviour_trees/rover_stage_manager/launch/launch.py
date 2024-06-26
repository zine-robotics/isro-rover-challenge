import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_stage_manager')

    socket_server_launch = Node(
        package='rover_stage_manager',
        executable='rover_server',
        output='screen'
    )

    # trajectory_action_server = Node(
    #     package='follow_joint_trajectory_server',
    #     executable='trajectory_action_server',
    #     output='screen',
    # )

    # joint_state_publisher = Node(
    #     package='follow_joint_trajectory_server',  # Replace with your package name
    #     executable='joint_state_publisher',  # Replace with your script name
    #     output='screen'
    # )

    behaviour_tree_launch = Node(
        package='rover_stage_manager', 
        executable='tree_node', 
        output='screen'
    )

    calibration_launch = Node(
        package="action_servers",
        executable="calibration",
        output='screen'
    )

    mandatory_wpf_launch = Node(
        package='action_servers', 
        executable='mandatory_wpf', 
        output='screen'
    )

    pickup_object_launch = Node(
        package='action_servers', 
        executable='pickup_object', 
        output='screen'
    )

    return LaunchDescription([
        socket_server_launch,
        # trajectory_action_server,
        # joint_state_publisher,
        calibration_launch,
        mandatory_wpf_launch,
        pickup_object_launch,
        behaviour_tree_launch,
    ])
