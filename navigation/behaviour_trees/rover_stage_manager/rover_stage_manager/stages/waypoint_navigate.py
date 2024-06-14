import py_trees
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup

from interfaces.action import Waypoint
from geometry_msgs.msg import Pose, PoseArray

class WaypointNavigate(py_trees.behaviour.Behaviour):
    def __init__(self, name="Navigate"):
        super(WaypointNavigate, self).__init__(name)
        self.action_client = None
        self.node = None
        self._goal_handle = None
        self._result = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node):
        self.logger.info("Waypoint Navigation Setup")

        self.node = node
        self.action_client = ActionClient(self.node, Waypoint, 'follow_waypoints')
        while not self.action_client.wait_for_server(timeout_sec=5):
            self.node.get_logger().error(f"{self.name} waiting for action server...")

    def initialise(self):
        self.node.get_logger().info("Waypoint Navigation Initialise")

        goal_msg = Waypoint.Goal()

        waypoints = PoseArray()
        if not self.blackboard.exists('missions'):
            waypoint_poses = [(2.95, -2.26), (2.88, 0.89)]
        else:
            missions = self.blackboard.get('missions')
            waypoint_poses = missions['SET_WAYPOINTS']['goal']

        self.node.get_logger().info(f"Waypoints Set To: {waypoint_poses}")
        for wp_x, wp_y in waypoint_poses:
            pose = Pose()
            pose.position.x = wp_x
            pose.position.y = wp_y
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            waypoints.poses.append(pose)
        
        goal_msg.waypoints = waypoints
        self._goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._goal_handle.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.error(f"{self.name} goal rejected")
            self._result = None
        else:
            self.logger.info(f"{self.name} goal accepted")
            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        self._result = future.result().result
        print("done", self._result)
        self.logger.info(f'Result: {self._result.success}')

        if not self._result.success:
            self.logger.error(f"{self.name} goal rejected")
            self._result = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pos = feedback.current_position
        self.logger.debug(f'Received feedback: {current_pos.x, current_pos.y} [{feedback.distance}m to goal]')

    def update(self):
        if self._result is not None:
            if self._result.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle:
            pass # print(dir(self._goal_handle))

    def shutdown(self):
        print("Destroy Waypoint Navigation")
        if self.action_client: 
            self.action_client.destroy()
