import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32MultiArray
import time
import math

class FollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_server')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'zine_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('FollowJointTrajectory action server has been started.')
        self._feedback = FollowJointTrajectory.Feedback()

        # Create a publisher to send trajectory points
        self.trajectory_publisher = self.create_publisher(Int32MultiArray, 'goal_array_topic', 10)

        # Create a subscriber to receive feedback
        self.feedback_subscriber = self.create_subscription(
            Int32MultiArray,
            'feedback_array_topic',
            self.feedback_callback,
            10
        )
        self._received_feedback: Int32MultiArray = None
        self.feedback_threshold = 2  # Threshold for feedback comparison

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = FollowJointTrajectory.Feedback()

        # Original order of joint names
        original_order: list = goal_handle.request.trajectory.joint_names
        self.get_logger().info(f'{original_order}')
        # Desired order of joint names
        desired_order = ['joint_0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # Create a mapping from the joint names to their indices in the original order
        mapping = {joint_name: original_order.index(joint_name) for joint_name in desired_order}

        # Get the last point in the trajectory
        point = goal_handle.request.trajectory.points[-1]

        # Initialize the trajectory message
        trajectory_msg = Int32MultiArray()

        # Reorder point.positions according to the mapping
        reordered_positions = [point.positions[mapping[joint_name]] for joint_name in desired_order]

        # Convert the reordered positions from radians to degrees and round to the nearest integer
        trajectory_msg.data = [int(math.degrees(pos)) for pos in reordered_positions]
        if trajectory_msg.data[5] > 0:
            trajectory_msg.data[5] = 1


        # Receive feedback from the subscribed topic
        while True:
            if goal_handle.is_cancel_requested:
                #send the same position as the feedback if goal is cancelled
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                rclpy.spin_once(self)
                if self._received_feedback is not None:
                    self.trajectory_publisher.publish(self._received_feedback)
                return FollowJointTrajectory.Result()

            self.trajectory_publisher.publish(trajectory_msg)
            self.get_logger().info(f'Sending trajectory point: {trajectory_msg.data}')
            rclpy.spin_once(self)  # Process any incoming messages
            if self._received_feedback is not None:
                position_errors = [abs(r - p) for r, p in zip(self._received_feedback.data, trajectory_msg.data)]
                if all(error <= self.feedback_threshold for error in position_errors):
                    break
                else:
                    self.get_logger().info(f'Waiting for feedback to be below threshold: {self._received_feedback.data}')
                # Use the received feedback
                feedback_msg.header.stamp = self.get_clock().now().to_msg()
                feedback_msg.joint_names = goal_handle.request.trajectory.joint_names
                feedback_msg.actual.positions = [math.radians(float(pos)) for pos in self._received_feedback.data]
                # self.get_logger().info(f'Received feedback: {self._received_feedback.data}')
                goal_handle.publish_feedback(feedback_msg)
        
        # goal_handle.success()
        self.get_logger().info('Goal succeeded!')
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        # result.success = True  # Assume success for simplicity
        return result

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received new goal.')
        return rclpy.action.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal has been canceled.')
        return rclpy.action.CancelResponse.ACCEPT

    def feedback_callback(self, msg):
        self.get_logger().info(f'Received feedback: {msg.data}')
        self._received_feedback = msg

def main(args=None):

    rclpy.init(args=args)
    node = FollowJointTrajectoryServer()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
