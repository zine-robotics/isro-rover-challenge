import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # nvblox_examples_dir = get_package_share_directory('nvblox_examples_bringup')
    zine_rover_dir = get_package_share_directory('zine_rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    manager_dir = get_package_share_directory('rover_stage_manager')
    hardware_dir = get_package_share_directory('hardware')

    stage_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            manager_dir, 'launch', 'manager.launch.py')
        ))

    zine_rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            zine_rover_dir, 'launch', 'gazebo.launch.py')
        ))
    
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            hardware_dir, 'launch', 'arm_hardware.launch.py')
        ))

    #NAV2 LAUNCH

   
    nav2_param_file = os.path.join(zine_rover_dir,'config', 'nav2.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'False',
                          'params_file': nav2_param_file,
                          'autostart': 'True'}.items())


    return LaunchDescription([
        # nav2_launch,
        stage_manager_launch,
        zine_rover_launch,
        
    ])
