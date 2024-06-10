import time
import rclpy
from rclpy.logging import get_logger

from moveit import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import GripperTrajectory

def main():
    ###################################################################
    # MoveIt Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_pick_place")

    # Create MoveGroupCommander and PlanningSceneInterface objects
    robot = MoveGroupCommander()
    scene = PlanningSceneInterface()

    # Define the reference frame for poses
    display_pose_frame = robot.get_planning_frame()

    ###################################################################
    # Define Pick and Place Parameters
    ###################################################################
    # Object pose for picking (adjust values for your specific scenario)
    object_pose = PoseStamped()
    object_pose.header.frame_id = display_pose_frame
    object_pose.pose.position.x = 0.2  # X-coordinate of the object
    object_pose.pose.position.y = 0.1  # Y-coordinate of the object
    object_pose.pose.position.z = 0.1  # Z-coordinate of the object (above the object)
    object_pose.pose.orientation.w = 1.0  # Neutral orientation

    # Approach pose for picking (slightly above the object pose)
    approach_pose = PoseStamped()
    approach_pose.header.frame_id = display_pose_frame
    approach_pose.pose.position = object_pose.pose.position
    approach_pose.pose.position.z += 0.05  # Move up slightly

    # Retreat pose for placing (slightly above the object pose)
    retreat_pose = PoseStamped()
    retreat_pose.header.frame_id = display_pose_frame
    retreat_pose.pose.position = object_pose.pose.position
    retreat_pose.pose.position.z += 0.1  # Move up further

    # Gripper pre-grasp pose (slightly open)
    gripper_pre_grasp_pose = GripperTrajectory()
    gripper_pre_grasp_pose.desired_minimum_effort = 0.1  # Adjust effort as needed
    gripper_pre_grasp_pose.desired_stopping_angle = 0.02  # Slightly open gripper

    # Gripper grasp pose (closed)
    gripper_grasp_pose = GripperTrajectory()
    gripper_grasp_pose.desired_minimum_effort = 0.5  # Adjust effort as needed
    gripper_grasp_pose.desired_stopping_angle = 0.0  # Fully closed gripper

###################################################################
# Planning and Execution
###################################################################

    # Add the object to the planning scene (replace with your object name)
    scene.add_box("my_object", object_pose, size=(0.1, 0.05, 0.05))

    # Move the end effector to approach pose
    robot.set_pose_target(approach_pose)
    plan = robot.go(wait=True)

    if plan is not None:
        logger.info("Moved to approach pose successfully")
    else:
        logger.error("Failed to move to approach pose")
        return

    # Set pre-grasp gripper state
    robot.execute(gripper_pre_grasp_pose, wait=True)

    # Move the end effector to pick pose
    robot.set_pose_target(object_pose)
    plan = robot.go(wait=True)

    if plan is not None:
        logger.info("Object picked successfully")
    else:
        logger.error("Failed to pick object")
        return

    # Set grasp gripper state
    robot.execute(gripper_grasp_pose, wait=True)

    # Move the end effector to retreat pose
    robot.set_pose_target(retreat_pose)
    plan = robot.go(wait=True)

    if plan is not None:
        logger.info("Moved to retreat pose successfully")
    else:
        logger.error("Failed to move to retreat pose")
        return

    # (Optional)
