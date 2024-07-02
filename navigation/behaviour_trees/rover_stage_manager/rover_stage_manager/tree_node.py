import py_trees
import py_trees_ros.trees
import rclpy

from rover_stage_manager.stages.waypoint_navigate import WaypointNavigate
from rover_stage_manager.stages.pickup_object import PickupObject
from rover_stage_manager.stages.drop_object import DropObject
from rover_stage_manager.stages.autonomous_navigation import AutonomousNavigate
from rover_stage_manager.stages.calibration import Calibrate
from rover_stage_manager.stages.base_station_setup import BaseStationSetup

class Finish(py_trees.behaviour.Behaviour):
    def __init__(self, name="Finish"):
        super(Finish, self).__init__(name)
        self.node = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, node):
        self.node = node

    def update(self):
        self.node.get_logger().info("FINISHED!")
        # self.node.get_logger().info(f"{self.blackboard.get('missions')}")
        return py_trees.common.Status.RUNNING
    

def create_root():
    root = py_trees.composites.Sequence("Root", True)
    
    root.add_children([
        BaseStationSetup(),
        Calibrate(), 
        WaypointNavigate(),
        PickupObject(),
        # AutonomousNavigate(),
        DropObject(),
        Finish()
    ])
    return root


def main():
    rclpy.init()

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(timeout=30)

    # tree.tick_tock(period_ms=1000.0)
    # try:
    #     rclpy.spin(tree.node)
    # except:
    #     pass
    # finally:
    #     tree.shutdown()
    #     rclpy.try_shutdown()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(tree.node)
    
    try:
        tree.tick_tock(1000.0)  # Tick the behaviour tree every 500 ms
        executor.spin()  # Allow ROS 2 to handle its callbacks
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        tree.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
