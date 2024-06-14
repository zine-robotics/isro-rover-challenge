import py_trees
from rclpy.node import Node
from rclpy.action import ActionClient

from interfaces.action import Calibration

class Calibrate(py_trees.behaviour.Behaviour):
    def __init__(self, name="Calibrate"):
        super(Calibrate, self).__init__(name)
        self.action_client = None
        self._goal_handle = None
        self._result = None
        self.node: Node = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node, **kwargs):
        self.node = node
        self.node.get_logger().info("Calibration Stage Setup")

        self.action_client = ActionClient(self.node, Calibration, 'calibrate')
        while not self.action_client.wait_for_server(timeout_sec=5):
            self.node.get_logger().error(f"{self.name} waiting for action server...")

    def initialise(self):
        self.node.get_logger().info("Calibration Stage Initialise")

        goal_msg = Calibration.Goal()
        self._goal_handle = self.action_client.send_goal_async(goal_msg)
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
        self._result = future.result().result
        self.node.get_logger().info(f'Result: {self._result.success}')

        if not self._result.success:
            self.node.get_logger().error(f"{self.name} goal rejected")
            self._result = None

    def update(self):
        # self.node.get_logger().info("Calibration Stage Update")
        if self._result is not None:
            if self._result.success:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
        # return py_trees.common.Status.SUCCESS