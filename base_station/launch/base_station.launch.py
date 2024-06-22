import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='zine_rover_description').find('zine_rover_description')
    default_model_path = os.path.join(pkg_share, 'urdf/zine_rover.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'config/base_station.rviz')
   
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # base_station_node = launch_ros.actions.Node(
    #     package='base_station',
    #     executable='base_station_gui',
    #     name='base_station_gui',
    #     output='screen',
    # )

 
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        rviz_node,
        # base_station_node,
    ])
