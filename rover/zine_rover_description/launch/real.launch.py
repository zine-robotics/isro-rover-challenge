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
    default_rviz_config_path = os.path.join(pkg_share, 'config/display.rviz')
    # world_path=os.path.join('/home/raman/workspaces/zine/src/zine_rover_description', 'world/arena_with_cylinder.sdf')


    twist_mux_params = os.path.join(pkg_share, 'config/twist_mux.yaml')
    twist_mux = launch_ros.actions.Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/demo/cmd_vel')]
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_wo_encoder.yaml')],
        remappings=[('/odometry/filtered','/odom')]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        robot_state_publisher_node,
        robot_localization_node,
        twist_mux,
    
    ])
