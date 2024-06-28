import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interfaces.action import Pickup
from geometry_msgs.msg import Pose

class PickupObjectActionClient(Node):
    def __init__(self):
        super().__init__('drop_object_action_client')
        self._action_client = ActionClient(self, Pickup, 'drop_object')
        self.get_logger().info('Pickup Object Action Client initialized')

    def send_goal(self):
        goal_msg = Pickup.Goal()
        # Populate the goal message with the object position
        object_position = Pose()
        object_position.position.x = 0.6  # Example position
        object_position.position.y = 0.0
        object_position.position.z = 0.0
        object_position.orientation.x = 0.0
        object_position.orientation.y = 0.0
        object_position.orientation.z = 0.0

        goal_msg.object_position = object_position

        self.get_logger().info('Sending goal request...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed')

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    action_client = PickupObjectActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
