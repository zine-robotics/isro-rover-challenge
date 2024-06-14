
#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
# from kinematic_solver import zine_ik_solver

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

import math
import numpy as np
# Segment lengths (in centimeters)
L1 = 10  # length of segment 1
L2 = 50  # length of segment 2
L3 = 40  # length of segment 3
L4 = 10  # length of segment 4
L5 = 0  # length of segment 5 (end effector)

# Joint limits (in degrees)
joint_limits = {
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

# Target end effector position (x, y, z) in centimeters
# target_x = 50
# target_y = 25
# target_z = 0

# # Fixed orientation of link1 (in degrees)
# theta1_fixed = 0  # assuming link1 is upright along the positive x-axis

def fix_quadrants(solution):
    limits = list(joint_limits.values())
    for i in range(len(solution)):
        if solution[i]>180:
            solution[i] = solution[i]  - 360
        elif solution[i]<-180:
            solution[i] = solution[i] + 360
    
    return solution
# Calculate inverse kinematics with joint limits
def inverse_kinematics(x, y, z):
    solutions = []

    # Joint 1 (Yaw) is fixed
    theta1 = math.atan2(y,x)

    # Adjust target position relative to link1's fixed orientation
    x_adj = x - L1 * math.cos(theta1)
    y_adj = y - L1 * math.sin(theta1)

    # Calculate distance from joint 1 to end effector in adjusted position
    distance = math.sqrt(x_adj**2 + y_adj**2 + z**2)

    # Check if the target is reachable
    if distance > L2 + L3 + L4 + L5:
        print("Target is out of reach!")
        return None

    # Iterate over possible solutions
    for i in range(-1, 2, 2):  # -1 and 1
        # Calculate theta5 (joint 5 angle, roll)
        theta5 = math.atan2(i * y_adj, i * x_adj)

        # Calculate distance from joint 4 to end effector in 2D plane (xy-plane)
        distance_xy = math.sqrt(x_adj**2 + y_adj**2)

        # Calculate theta3 (joint 3 angle)
        cos_theta3 = (distance_xy**2 + (z - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(cos_theta3) > 1:
            continue  # skip invalid solutions
        theta3_1 = math.acos(cos_theta3)
        theta3_2 = -theta3_1  # second possible solution

        # Iterate over theta3 solutions
        for theta3 in [theta3_1, theta3_2]:
            # Calculate theta2 (joint 2 angle)
            theta2 = math.atan2(z - L1, distance_xy) \
                     - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))

            # Calculate theta4 (joint 4 angle, roll)
            theta4 = theta5 - theta2 - theta3

            # Calculate theta6 (joint 6 angle for end effector)
            theta6 = math.atan2(z - L1, distance_xy)

            # Check joint limits for all angles
            if (joint_limits['theta1_min'] <= math.degrees(theta1) <= joint_limits['theta1_max']) and \
               (joint_limits['theta2_min'] <= math.degrees(theta2 )<= joint_limits['theta2_max']) and \
               (joint_limits['theta3_min'] <= math.degrees(theta3) <= joint_limits['theta3_max']) and \
               (joint_limits['theta4_min'] <= math.degrees(theta4) <= joint_limits['theta4_max']) and \
               (joint_limits['theta5_min'] <= math.degrees(theta5) <= joint_limits['theta5_max']) and \
               (joint_limits['theta6_min'] <= math.degrees(theta6) <= joint_limits['theta6_max']):
                solutions.append((theta1, theta2, theta3, theta4, theta5, theta6))

    return solutions

# Perform inverse kinematics calculation

def zine_ik_solver(target_x, target_y, target_z):
    kinematic_solutions = inverse_kinematics(target_x, target_y, target_z)
    if kinematic_solutions:
        # Find the best solution (closest to initial position)
        best_solution = None
        min_distance = float('inf')
    
        for solution in kinematic_solutions:
            # Calculate distance from initial position to end effector position of the solution
            x_end = L1 * math.cos(solution[0]) + L2 * math.cos(solution[0] + solution[1]) \
                    + L3 * math.cos(solution[0] + solution[1] + solution[2]) \
                    + L4 * math.cos(solution[0] + solution[1] + solution[2] + solution[3]) \
                    + L5 * math.cos(solution[0] + solution[1] + solution[2] + solution[3] + solution[4])
            y_end = L1 * math.sin(solution[0]) + L2 * math.sin(solution[0] + solution[1]) \
                    + L3 * math.sin(solution[0] + solution[1] + solution[2]) \
                    + L4 * math.sin(solution[0] + solution[1] + solution[2] + solution[3]) \
                    + L5 * math.sin(solution[0] + solution[1] + solution[2] + solution[3] + solution[4])
            z_end = L1 + L2 * math.cos(solution[1]) + L3 * math.cos(solution[1] + solution[2]) \
                    + L4 * math.cos(solution[1] + solution[2] + solution[3]) \
                    + L5 * math.cos(solution[1] + solution[2] + solution[3] + solution[4])
    
            distance = math.sqrt((target_x - x_end)**2 + (target_y - y_end)**2 + (target_z - z_end)**2)
    
            # Update best solution if closer to the target position
            if distance < min_distance:
                min_distance = distance
                best_solution = solution
        best_solution = np.degrees(np.array(best_solution))
        best_solution[1] -= 90
        best_solution[2] += 90
        best_solution = fix_quadrants(best_solution)
        best_solution[3] = 0.0
        best_solution[4] = 0.0
        # best_solution[4] = best_solution[1] + best_solution[2]

        return np.radians(best_solution[:5])

    else:
        return [0,0,0,0,0]
    

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
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
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def move_to_pose(zine_rover,zine_arm,logger,pose):
    
    zine_arm.set_start_state_to_current_state()
    robot_model = zine_rover.get_robot_model()
    robot_state = RobotState(robot_model)
    
    x = pose[0]
    y = pose[1]
    z = pose[2]

    joint_ik_values = zine_ik_solver(x,y,z)
    joint_values = {
        "joint_0": joint_ik_values[0],
        "joint1": joint_ik_values[1],
        "joint2": joint_ik_values[2],
        "joint3": joint_ik_values[3],
        "joint4": joint_ik_values[4],
    }

    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=zine_rover.get_robot_model().get_joint_model_group("zine_arm"),
    )
    zine_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    plan_and_execute(zine_rover, zine_arm, logger, sleep_time=5)


def main():


    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    zine_rover = MoveItPy(node_name="moveit_py")
    zine_arm = zine_rover.get_planning_component("zine_arm")
    logger.info("MoveItPy instance created")

    POSE = [50,30,-5]
    POSE[2]+=10
    move_to_pose(zine_rover,zine_arm,logger,POSE)
    logger.info("Arm reached for approaching position")
    time.sleep(5)
    POSE[2]-=10
    move_to_pose(zine_rover,zine_arm,logger,POSE)
    logger.info("Arm reached for gripping position")
    POSE[2]+=40
    move_to_pose(zine_rover,zine_arm,logger,POSE)
    logger.info("Arm reached for safe position")



if __name__ == "__main__":
    main()