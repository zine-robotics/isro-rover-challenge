import math
import numpy as np

# Lengths of the robotic arm segments
L1 = 30  # length of segment 1 (fixed on the z-axis)
L2 = 55  # length of segment 2
L3 = 55  # length of segment 3

# Joint limits (in degrees)
joint_limits = {
    'theta1_min': -180,
    'theta1_max': 180,
    'theta2_min': 0,
    'theta2_max': 180,
    'theta3_min': -90,
    'theta3_max': 90,
}

def inverse_kinematics(x, y, z):
    solutions = []

    # Joint 1 (Yaw)
    theta1 = math.atan2(y, x)

    # Adjust target position relative to the base of the second link
    r = math.sqrt(x**2 + y**2)
    z_adj = z - L1
    d = math.sqrt(r**2 + z_adj**2)

    # Check if the target is reachable
    if d > L2 + L3 or d < abs(L2 - L3):
        print("Target is out of reach!")
        return None

    # Calculate theta3 (elbow angle)
    cos_theta3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if abs(cos_theta3) > 1:
        print("No valid solution for theta3")
        return None

    theta3_1 = math.acos(cos_theta3)
    theta3_2 = -theta3_1  # second possible solution

    # Iterate over theta3 solutions
    for theta3 in [theta3_1, theta3_2]:
        # Calculate theta2 (shoulder angle)
        theta2 = math.atan2(z_adj, r) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))

        # Check joint limits for all angles
        if (joint_limits['theta1_min'] <= math.degrees(theta1) <= joint_limits['theta1_max']) and \
           (joint_limits['theta2_min'] <= math.degrees(theta2) <= joint_limits['theta2_max']) and \
           (joint_limits['theta3_min'] <= math.degrees(theta3) <= joint_limits['theta3_max']):
            solutions.append((theta1, theta2, theta3))

    return solutions

def best_solution_ik_solver(target_x, target_y, target_z):
    kinematic_solutions = inverse_kinematics(target_x, target_y, target_z)
    if kinematic_solutions:
        # Find the best solution (closest to initial position)
        best_solution = None
        min_distance = float('inf')

        for solution in kinematic_solutions:
            # Calculate end effector position of the solution
            x_end = (L2 * math.cos(solution[1]) + L3 * math.cos(solution[1] + solution[2])) * math.cos(solution[0])
            y_end = (L2 * math.cos(solution[1]) + L3 * math.cos(solution[1] + solution[2])) * math.sin(solution[0])
            z_end = L1 + L2 * math.sin(solution[1]) + L3 * math.sin(solution[1] + solution[2])

            distance = math.sqrt((target_x - x_end)**2 + (target_y - y_end)**2 + (target_z - z_end)**2)

            # Update best solution if closer to the target position
            if distance < min_distance:
                min_distance = distance
                best_solution = solution

        best_solution = np.degrees(best_solution)
        final_solution = np.zeros(5)
        final_solution[0] = best_solution[0]
        final_solution[1] = best_solution[1]-90
        final_solution[2] = best_solution[2]+90
        final_solution[4] = final_solution[1] + final_solution[2]
        if final_solution[4]<-85: final_solution[4]=-85
        elif final_solution[4]>85: final_solution[4]=85
        final_solution = fix_quadrants(final_solution)
        return final_solution
    
    else:
        return [0.0, 0.0, 0.0,0.0,0.0]

def fix_quadrants(solution):
    limits = list(joint_limits.values())
    for i in range(len(solution)):
        if solution[i]>180:
            solution[i] = solution[i]  - 360
        elif solution[i]<-180:
            solution[i] = solution[i] + 360
    
    return solution
# Example usage
target_x, target_y, target_z = 80, 0, -15
solution = best_solution_ik_solver(target_x, target_y, target_z)
print("Best joint angles (degrees):", solution)
