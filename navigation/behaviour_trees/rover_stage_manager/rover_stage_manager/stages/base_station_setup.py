import py_trees
from rclpy.node import Node
from std_msgs.msg import String
import json

class BaseStationSetup(py_trees.behaviour.Behaviour):
    def __init__(self, name="Base Station Setup"):
        super(BaseStationSetup, self).__init__(name)
        self.subscriber = None
        self.node: Node = None
        self.is_setup_complete = False
        self.first_update = True
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node, **kwargs):
        self.node = node
        self.node.get_logger().info("Base Station Stage Setup")

        self.subscriber = self.node.create_subscription(
            String,
            'base_station_message',
            self.got_message_callback,
            10
        )

    def initialise(self):
        self.node.get_logger().info("Base Station Stage Initialise")

    def got_message_callback(self, msg):
        self.node.get_logger().info("Received message from base station")
        data = json.loads(msg.data)

        missions = {}
        missions_data = data['mission']
        self.node.get_logger().info(f"{missions_data}")
        for m in missions_data:
            missions[m['name']] = {'goal': m['goal']}

        self.blackboard.set('map', data['map'])
        self.blackboard.set('missions', missions)

        self.is_setup_complete = True

    def update(self):
        if self.first_update:
            self.node.get_logger().info("Waiting for server response")
            self.first_update = False

        if self.is_setup_complete:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING