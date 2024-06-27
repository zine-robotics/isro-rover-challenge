import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray
import time

class TestFollowJointTrajectoryClient(Node):

    def __init__(self):
        super().__init__('test_follow_joint_trajectory_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, 'zine_arm_controller/follow_joint_trajectory')
        
    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        
        # Define the trajectory points
        point = JointTrajectoryPoint()
        # point.positions = [-0.523599, 0.523599, -0.523599, -0.523599, -0.523599]
        point.positions = [0.0, -1.535, 0.0, 0.0, 0.0,0.0]
        point.time_from_start.sec = 1

        goal_msg.trajectory.points.append(point)
        goal_msg.trajectory.joint_names = ['joint_0', 'joint1', 'joint2', 'joint3', 'joint4','joint5']
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.actual.positions}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')

def main(args=None):
    rclpy.init(args=args)
    client = TestFollowJointTrajectoryClient()
    client.send_goal()
    
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
