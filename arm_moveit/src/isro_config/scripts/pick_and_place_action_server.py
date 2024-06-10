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

from moveit.planning import MoveItPy

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup

class MoveItActionServer(Node):
    def __init__(self):
        super().__init__('moveit_action_server')
        self.logger = get_logger('moveit_action_server')
        
        self.moveit_py = MoveItPy(node_name='moveit_py_planning_scene')
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
    
    def pose_callback(self, msg):
        self.object_pose = msg.pose
    
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
