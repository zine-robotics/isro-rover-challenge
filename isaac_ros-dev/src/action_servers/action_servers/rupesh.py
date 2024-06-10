#!/usr/bin/env python3
"""
Action server using MoveItPy to plan and execute robot motions based on the pose of an object published by a node.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup

class MoveItActionServer(Node):
    def __init__(self):
        super().__init__('moveit_action_server')
        self.logger = get_logger('moveit_action_server')
        
        self.moveit_py = MoveItPy(node_name='moveit_py_planning_scene_zine')
        self.panda_arm = self.moveit_py.get_planning_component('arm')
        self.planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()
        
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'object_pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.action_server = ActionServer(
            self,
            MoveGroup,
            'move_group',
            self.execute_callback
        )

        self.logger.info('MoveItPy instance and action server created')

    
    def go_to_pose(self):
        self.logger.info("Go to goal")
        with self.planning_scene_monitor.read_write() as scene:

            # instantiate a RobotState instance using the current robot model
            robot_state = scene.current_state
            original_joint_positions = robot_state.get_joint_group_positions("arm")
            print(original_joint_positions)
            robot_state.update()

            check_init_pose = robot_state.get_pose("limb5_1")
            self.logger.info("Initial_pose: {check_init_pose}")

            pose_goal = Pose()
            pose_goal.position.x =  0.9
            pose_goal.position.y = 0.02
            pose_goal.position.z = 0.8
            pose_goal.orientation.x = 0.1
            pose_goal.orientation.y =  0.21
            pose_goal.orientation.z = .343
            pose_goal.orientation.w = .99
            self.logger.info("Offset_pose: {pose_goal}")


            # randomize the robot state

            result = robot_state.set_from_ik(joint_model_group_name="arm",
                geometry_pose=pose_goal,
                tip_name="limb5_1",
                timeout=8.0)  # Increase the timeout

# Check the result of the IK solver
            if not result:
                self.logger.error("IK solution was not found!")
                return

            robot_state.update()
            robot_state.set_to_random_positions()

            check_updated_pose = robot_state.get_pose("limb5_1")

            self.logger.info("New_pose: {check_updated_pose}")


            # set goal state to the initialized robot state
            self.logger.info("Go to goal")
            self.panda_arm.set_goal_state(robot_state=robot_state)
           
            robot_state.update()

         # plan to goal
        self.logger.info("Going to goal")
        self.plan_and_execute()
        time.sleep(3)
    
    def pose_callback(self, msg):
        self.object_pose = msg.pose
      
        # self.panda_arm.set_goal_state(configuration_name="ready")
        # self.plan_and_execute()
        # current_pose = self.panda_arm.get_start_state()
        # self.logger.info(f'Current pose: {current_pose}')
        self.go_to_pose()

        #Define the goal pose based on the current pose
        # pose_goal = PoseStamped()
        # pose_goal.header.frame_id = "arm"
        # pose_goal.pose.orientation.w = 0.99
        # pose_goal.pose.orientation.z = -0.116
        # pose_goal.pose.orientation.y = 0.0
        # pose_goal.pose.orientation.x = 0.08
        # pose_goal.pose.position.x =  0.43
        # pose_goal.pose.position.y = -0.05
        # pose_goal.pose.position.z = 0.412

        # # Set the goal pose for the end effector
        # self.panda_arm.set_start_state_to_current_state()
        # self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal,pose_link= "limb6_1")
        # self.plan_and_execute()
        # robot_model = self.moveit_py.get_robot_model()
        # robot_state = RobotState(robot_model)

        # # randomize the robot state
        # robot_state.set_to_random_positions()

        # # set plan start state to current state
        # self.panda_arm.set_start_state_to_current_state()

        # # set goal state to the initialized robot state
        # self.logger.info("Set goal state to the initialized robot state")
        # self.panda_arm.set_goal_state(robot_state=robot_state)

        # # plan to goal
        # self.plan_and_execute()
        time.sleep(1)

            

        
    

    

        
        
    def execute_callback(self, goal_handle):
        self.logger.info('Executing goal...')

        if self.is_within_distance(self.object_pose):
            self.panda_arm.set_start_state_to_current_state()
            self.panda_arm.set_goal_state_from_pose(self.object_pose, 'hand')
            self.plan_and_execute()
            goal_handle.succeed()
            result = MoveGroup.Result()
            result.trajectory = self.panda_arm.get_current_trajectory()
            return result
        else:
            self.logger.info('Object is not within 1 meter, ignoring goal.')
            goal_handle.abort()
    
    def is_within_distance(self, pose, threshold=1.0):
        distance = (pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2) ** 0.5
        return distance <= threshold

    def plan_and_execute(self):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        plan_result = self.panda_arm.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.moveit_py.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(0.0)
    


def main(args=None):
    rclpy.init(args=args)
    moveit_action_server = MoveItActionServer()
    rclpy.spin(moveit_action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()