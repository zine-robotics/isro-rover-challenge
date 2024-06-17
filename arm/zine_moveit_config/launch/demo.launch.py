from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "zine_rover", 
        package_name="zine_moveit_config"
        ).to_moveit_configs()



    return generate_demo_launch(moveit_config)
