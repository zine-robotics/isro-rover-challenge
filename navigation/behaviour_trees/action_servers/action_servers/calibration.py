import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse, server
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from interfaces.action import Calibration


class CalibrationActionServer(Node):
    def __init__(self):
        super().__init__('calibration_action_server')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10)
        )

        self.action_server = ActionServer(
            self,
            Calibration,
            'calibrate',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory'
        )

        self.is_data_received = False
        self._goal_handle = None
        self.get_logger().info('Initialized Calibration Action Server')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received calibration goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        # Update flag when odometry data is received
        self.is_data_received = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    async def execute_callback(self, goal_handle: server.ServerGoalHandle):
        self.get_logger().info('Executing calibration...')
        self._goal_handle = goal_handle

        self.get_logger().info('1. Check odom')
        while not self.is_data_received and self._goal_handle is not None:
            rclpy.spin_once(self)
        
        self.get_logger().info('Success: Odom Received!')

        
        self.get_logger().info('2. Move arm to origin')
        joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]  # Example joint angles, adjust as needed
        trajectory_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ['Revolute 19', 'Revolute 13', 'Revolute 14', 'Revolute 15', 'Revolute 16']
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 5  # Example duration, adjust as needed
        trajectory.points.append(point)
        trajectory_msg.trajectory = trajectory

        # self.get_logger().info('Started moving arm...')
        # self.joint_trajectory_client.wait_for_server()
        # send_goal_future = self.joint_trajectory_client.send_goal_async(trajectory_msg, feedback_callback=self.feedback_callback)
        # rclpy.spin_until_future_complete(self, send_goal_future)

        # goal_handle = send_goal_future.result()
        # if not goal_handle.accepted:
        #     self.get_logger().info('Joint trajectory goal rejected')
        #     self._goal_handle.abort()
        #     return Calibration.Result(success=False)

        # get_result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, get_result_future)

        # self.get_logger().info('Success: Arm moved to origin')

        result = Calibration.Result()
        result.success = True
        self.get_logger().info('Calibration successful')
        # if get_result_future.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
        #     self.get_logger().info('Calibration successful')
        #     result.success = True
        # else:
        #     self.get_logger().info('Calibration failed')
        #     result.success = False
        self._goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    calibration_action_server = CalibrationActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(calibration_action_server, executor=executor)
    calibration_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
