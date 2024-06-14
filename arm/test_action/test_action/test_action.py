#!/opt/anaconda3/bin python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interfaces.action import Waypoint
from geometry_msgs.msg import Pose, PoseArray

class WaypointActionClient(Node):

    def __init__(self):
        super().__init__('test_action')
        self._action_client = ActionClient(self, Waypoint, 'follow_waypoints')
        
    def send_goal(self, waypoint):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Waypoint.Goal()
        # print(dir(goal_msg))
        goal_msg.waypoints = waypoint

        self.get_logger().info('Sending goal...')
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

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pos = feedback.current_position
        self.get_logger().info(f'Received feedback: {current_pos.x, current_pos.y} [{feedback.distance}m to goal]')

def main(args=None):
    rclpy.init(args=args)

    action_client = WaypointActionClient()

    # Create a dummy waypoint pose
    waypoints = PoseArray()
    for i in range(1, 3):
        pose = Pose()
        pose.position.x = 0.5*i
        pose.position.y = -0.5*i
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        waypoints.poses.append(pose)

    action_client.send_goal(waypoints)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
