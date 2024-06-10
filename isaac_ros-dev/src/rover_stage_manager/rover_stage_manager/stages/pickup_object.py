import py_trees
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup

from interfaces.action import Waypoint

class PickupObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="Pickup Object"):
        super(PickupObject, self).__init__(name)
        self.action_client = None
        self.node = None
        self._goal_handle = None
        self._result = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node):
        self.node = node
        self.node.get_logger().info("Pickup Object Setup")

        # TODO:
        # # Put Action Type and Action String
        # self.action_client = ActionClient(self.node, Waypoint, 'follow_waypoints')
        # ##

        # while not self.action_client.wait_for_server(timeout_sec=5):
        #     self.logger.error(f"{self.name} waiting for action server...")

    def initialise(self):
        self.node.get_logger().info("Pickup Object Initialize")
        # TODO:
        # goal_msg = Waypoint.Goal()
        # # Fill Goal Message 
        # ##

        # self._goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # self._goal_handle.add_done_callback(self._goal_response_callback)

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
        self.logger.info(f'Result: {self._result.success}')

        if not self._result.success:
            self.logger.error(f"{self.name} goal rejected")
            self._result = None

    def feedback_callback(self, feedback_msg):
        # Edit Feedback output
        feedback = feedback_msg.feedback
        current_pos = feedback.current_position
        self.logger.debug(f'Received feedback: {current_pos.x, current_pos.y} [{feedback.distance}m to goal]')
        ##

    def update(self):
        # TODO: 
        # if self._result is not None:
        #     if self._result.success:
        #         return py_trees.common.Status.SUCCESS
        #     else:
        #         return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle:
            pass

    def shutdown(self):
        print("destroy")
        self.action_client.destroy()
