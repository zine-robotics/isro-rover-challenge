import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  
    map_node = launch_ros.actions.Node(
            package="map_server",
            executable="map_node",
    )


    return launch.LaunchDescription([
       map_node,
    
    ])
