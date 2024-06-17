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
import math
from moveit.planning import MoveItPy
import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint


class MoveItActionServer(Node):
    def __init__(self):
        super().__init__('moveit_action_server')
        self.logger = get_logger('moveit_action_server')
        
        self.zine_rover = MoveItPy(node_name='moveit_py_planning_scene_zine')
        self.zine_arm = self.zine_rover.get_planning_component('arm')
        self.planning_scene_monitor = self.zine_rover.get_planning_scene_monitor()
        
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
        # Segment lengths (in centimeters)
        self.L1 = 10  # length of segment 1
        self.L2 = 50  # length of segment 2
        self.L3 = 40  # length of segment 3
        self.L4 = 10  # length of segment 4
        self.L5 = 0  # length of segment 5 (end effector)

        # Joint limits (in degrees)
        self.joint_limits = {
            'theta1_min': -180,
            'theta1_max': 180,
            'theta2_min': 0,
            'theta2_max': 180,
            'theta3_min': -160,
            'theta3_max': 160,
            'theta4_min': -180,
            'theta4_max': 180,
            'theta5_min': -90,
            'theta5_max': 90,
            'theta6_min': -180,
            'theta6_max': 180,
        }

        self.logger.info('MoveItPy instance and action server created')
    
    def fix_quadrants(self,solution):
        limits = list(self.self.joint_limits.values())
        for i in range(len(solution)):
            if solution[i]>180:
                solution[i] = solution[i]  - 360
            elif solution[i]<-180:
                solution[i] = solution[i] + 360
        
        return solution
    # Calculate inverse kinematics with joint limits
    def inverse_kinematics(self,x, y, z):
        solutions = []

        # Joint 1 (Yaw) is fixed
        theta1 = math.atan2(y,x)

        # Adjust target position relative to link1's fixed orientation
        x_adj = x - self.L1 * math.cos(theta1)
        y_adj = y - self.L1 * math.sin(theta1)

        # Calculate distance from joint 1 to end effector in adjusted position
        distance = math.sqrt(x_adj**2 + y_adj**2 + z**2)

        # Check if the target is reachable
        if distance > self.L2 + self.L3 + self.L4 + self.L5:
            print("Target is out of reach!")
            return None

        # Iterate over possible solutions
        for i in range(-1, 2, 2):  # -1 and 1
            # Calculate theta5 (joint 5 angle, roll)
            theta5 = math.atan2(i * y_adj, i * x_adj)

            # Calculate distance from joint 4 to end effector in 2D plane (xy-plane)
            distance_xy = math.sqrt(x_adj**2 + y_adj**2)

            # Calculate theta3 (joint 3 angle)
            cos_theta3 = (distance_xy**2 + (z - self.L1)**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            if abs(cos_theta3) > 1:
                continue  # skip invalid solutions
            theta3_1 = math.acos(cos_theta3)
            theta3_2 = -theta3_1  # second possible solution

            # Iterate over theta3 solutions
            for theta3 in [theta3_1, theta3_2]:
                # Calculate theta2 (joint 2 angle)
                theta2 = math.atan2(z - self.L1, distance_xy) \
                        - math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))

                # Calculate theta4 (joint 4 angle, roll)
                theta4 = theta5 - theta2 - theta3

                # Calculate theta6 (joint 6 angle for end effector)
                theta6 = math.atan2(z - self.L1, distance_xy)

                # Check joint limits for all angles
                if (self.joint_limits['theta1_min'] <= math.degrees(theta1) <= self.joint_limits['theta1_max']) and \
                (self.joint_limits['theta2_min'] <= math.degrees(theta2 )<= self.joint_limits['theta2_max']) and \
                (self.joint_limits['theta3_min'] <= math.degrees(theta3) <= self.joint_limits['theta3_max']) and \
                (self.joint_limits['theta4_min'] <= math.degrees(theta4) <= self.joint_limits['theta4_max']) and \
                (self.joint_limits['theta5_min'] <= math.degrees(theta5) <= self.joint_limits['theta5_max']) and \
                (self.joint_limits['theta6_min'] <= math.degrees(theta6) <= self.joint_limits['theta6_max']):
                    solutions.append((theta1, theta2, theta3, theta4, theta5, theta6))

        return solutions
    def zine_ik_solver(self,target_x, target_y, target_z):
        kinematic_solutions = self.inverse_kinematics(target_x, target_y, target_z)
        if kinematic_solutions:
            # Find the best solution (closest to initial position)
            best_solution = None
            min_distance = float('inf')
        
            for solution in kinematic_solutions:
                # Calculate distance from initial position to end effector position of the solution
                x_end =  self.L1 * math.cos(solution[0]) + self.L2 * math.cos(solution[0] + solution[1]) \
                        + self.L3 * math.cos(solution[0] + solution[1] + solution[2]) \
                        + self.L4 * math.cos(solution[0] + solution[1] + solution[2] + solution[3]) \
                        + self.L5 * math.cos(solution[0] + solution[1] + solution[2] + solution[3] + solution[4])
                y_end = self.L1 * math.sin(solution[0]) + self.L2 * math.sin(solution[0] + solution[1]) \
                        + self.L3 * math.sin(solution[0] + solution[1] + solution[2]) \
                        + self.L4 * math.sin(solution[0] + solution[1] + solution[2] + solution[3]) \
                        + self.L5 * math.sin(solution[0] + solution[1] + solution[2] + solution[3] + solution[4])
                z_end = self.L1 + self.L2 * math.cos(solution[1]) + self.L3 * math.cos(solution[1] + solution[2]) \
                        + self.L4 * math.cos(solution[1] + solution[2] + solution[3]) \
                        + self.L5 * math.cos(solution[1] + solution[2] + solution[3] + solution[4])
        
                distance = math.sqrt((target_x - x_end)**2 + (target_y - y_end)**2 + (target_z - z_end)**2)
        
                # Update best solution if closer to the target position
                if distance < min_distance:
                    min_distance = distance
                    best_solution = solution
            best_solution = np.degrees(np.array(best_solution))
            best_solution[1] -= 90
            best_solution[2] += 90
            best_solution = self.fix_quadrants(best_solution)
            best_solution[3] = 0.0
            best_solution[4] = 0.0
            # best_solution[4] = best_solution[1] + best_solution[2]

            return np.radians(best_solution[:5])

        else:
            return [0,0,0,0,0]
    def pose_callback(self, msg):
        self.object_pose = msg.pose
        self.logger.info('Got point')
        POSE={'x':20,'y':10,'z':8}
        self.move_to_pose(POSE)
        self.logger.info("Arm reached for approaching position")
       
    
    def execute_callback(self, goal_handle):
        self.logger.info('Executing goal...')
        theta1 = math.atan2(self.object_pose.y,self.object_pose.x)
        x_adj = self.object_pose.x - self.L1 * math.cos(theta1)
        y_adj = self.object_pose.y - self.L1 * math.sin(theta1)

    # Calculate distance from joint 1 to end effector in adjusted position
        distance = math.sqrt(x_adj**2 + y_adj**2 + self.object_pose.z**2)
        if distance > self.L2 + self.L3 + self.L4 + self.L5:
            POSE={'x':20,'y':10,'z':8}
            self.move_to_pose(POSE)
            self.logger.info("Arm reached for approaching position")
            time.sleep(5)
            result = MoveGroup.Result()
            result.trajectory = self.zine_arm.get_current_trajectory()
            return result
        else:
            self.logger.info('Object is not within  range, ignoring goal.')
            goal_handle.abort()
    

    def plan_and_execute(
        self,
        robot,
        planning_component,
        
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(sleep_time)
    
    def move_to_pose(self,pose):
        
        self.zine_arm.set_start_state_to_current_state()
        robot_model = self.zine_rover.get_robot_model()
        robot_state = RobotState(robot_model)
        
        x = pose['x']
        y = pose['y']
        z = pose['z']

        joint_ik_values = self.zine_ik_solver(x,y,z)
        joint_values = {

            "Slider 17": joint_ik_values[0],
            "Revolute 19": joint_ik_values[1],
            "Revolute 13": joint_ik_values[2],
            "Revolute 14": joint_ik_values[3],
            "Revolute 13": joint_ik_values[4],
        }

        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=self.zine_rover.get_robot_model().get_joint_model_group("arm"),
        )
        self.zine_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        self.plan_and_execute(self.zine_rover, self.zine_arm, sleep_time=5)


def main(args=None):
    rclpy.init(args=args)
    moveit_action_server = MoveItActionServer()
    rclpy.spin(moveit_action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
