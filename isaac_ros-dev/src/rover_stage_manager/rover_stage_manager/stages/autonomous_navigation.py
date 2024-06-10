import py_trees
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class AutonomousNavigate(py_trees.behaviour.Behaviour):
    def __init__(self, name="AutonomousNavigate"):
        super(AutonomousNavigate, self).__init__(name)
        self.action_client = None
        self._goal_handle = None
        self._result = None
        self.node: Node = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node):
        self.node = node
        self.node.get_logger().info("Autonomous Navigation Setup")

        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        while not self.action_client.wait_for_server(timeout_sec=5):
            self.logger.error(f"{self.name} waiting for action server...")

    def initialise(self):
        self.node.get_logger().info("Autonomous Navigation Initialise")

        if not self.blackboard.exists('missions'):
            goal_pose = (9.0, -3.0)
        else:
            missions = self.blackboard.get('missions')
            goal_pose = missions['AUTONOMOUS_NAVIGATE']['goal']
    
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = float(goal_pose[0])
        pose.pose.position.y = float(goal_pose[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose

        self._goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._goal_handle.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().error(f"{self.name} goal rejected")
            self._result = None
        else:
            self.node.get_logger().info(f"{self.name} goal accepted")
            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        self._result = future.result()
        print("done", self._result)
        self.logger.info(f'Result: {self._result}')

        if not self._result:
            self.logger.error(f"{self.name} goal rejected")
            self._result = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pos = feedback.current_pose
        # self.node.get_logger().info(f"Feedback: {dir(current_pos)}")
        self.node.get_logger().info(f'Received feedback: {current_pos.pose.position.x, current_pos.pose.position.y} [{feedback.distance_remaining}m to goal]')

    def update(self):
        if self._result is not None:
            return py_trees.common.Status.SUCCESS if self._result else py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle:
            self._goal_handle.cancel_goal_async()