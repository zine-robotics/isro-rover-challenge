from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("isroarm", package_name="isro_config").to_moveit_configs()
    moveit_config.move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
    return generate_move_group_launch(moveit_config)
