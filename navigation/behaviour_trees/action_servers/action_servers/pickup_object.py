import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from interfaces.action import Pickup 
import math
from rclpy.qos import QoSProfile

class PickupObjectActionServer(Node):

    def __init__(self):
        super().__init__('pickup_object')
        # self.subscription = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_callback,
        #     QoSProfile(depth=10))
        # self.publisher = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))
        self.action_server = ActionServer(
            self,
            Pickup,
            'pickup_object',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.current_position = None
        self.current_orientation = None
        self.tolerance = 0.1
        self._goal_handle = None
        self._waypoints = []
        self._current_waypoint_index = 0

        self.get_logger().info('Initialized Pick Up Object Action Server')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def move_to_waypoint(self):
        target_x, target_y = self._waypoints[self._current_waypoint_index]

        # if distance < self.tolerance:
        #     self.get_logger().info(f'{distance} | {self.current_position} | {(target_x, target_y)}')
        #     self.get_logger().info(f'Reached waypoint {self._current_waypoint_index + 1}')
        #     self._current_waypoint_index += 1
        #     if self._current_waypoint_index >= len(self._waypoints):
        #         self._goal_handle.succeed()
        #         self._goal_handle = None

        #         # Stopppp
        #         cmd_vel = Twist()
        #         self.publisher.publish(cmd_vel)
        #         self.get_logger().info(f'Finished waypoint navigation {self.current_position}')

        #     return

        # cmd_vel = Twist()
        # self.publisher.publish(cmd_vel)

        # feedback_msg = Waypoint.Feedback()
        # feedback_msg.current_position = Point(x=self.current_position[0], y=self.current_position[1], z=0.0)
        # feedback_msg.distance = distance
        # self._goal_handle.publish_feedback(feedback_msg)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle

        self.object_position = goal_handle.request.object_position


        while self._goal_handle is not None:
            rclpy.spin_once(self)
            # Some break condition pls

            # if self._current_waypoint_index >= len(self._waypoints):
            #     break
        
        result = Pickup.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    pickup_action_server = PickupObjectActionServer()
    rclpy.spin(pickup_action_server)
    pickup_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()