import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nvblox_examples_dir = get_package_share_directory('nvblox_examples_bringup')
    manager_dir = get_package_share_directory('rover_stage_manager')


    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nvblox_examples_dir, 'launch', 'isaac_sim_example.launch.py')
        ))

    stage_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            manager_dir, 'launch', 'launch.py')
        ))

    return LaunchDescription([
        nvblox_launch,
        stage_manager_launch
    ])
