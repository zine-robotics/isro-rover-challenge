from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("isroarm", package_name="isro_config").moveit_cpp(
            file_path=get_package_share_directory("isro_config")
            + "/config/path_planning_api.yaml"
        ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
