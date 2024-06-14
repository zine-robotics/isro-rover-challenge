import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from interfaces.action import Waypoint  # Assuming you have a custom message package 'custom_msgs'
import math
from rclpy.qos import QoSProfile

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('mandatory_waypoint_follower')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            QoSProfile(depth=10))
        self.publisher = self.create_publisher(Twist, 'stage1_cmd_vel', QoSProfile(depth=10))
        self.action_server = ActionServer(
            self,
            Waypoint,
            'follow_waypoints',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.current_position = None
        self.current_orientation = None
        self.tolerance = 0.1
        self._goal_handle = None
        self._waypoints = []
        self._current_waypoint_index = 0

        self.get_logger().info('Initialized Mandatory Waypoint Follower Action Server')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_orientation = yaw

        if self._goal_handle is not None and self._current_waypoint_index < len(self._waypoints):
            self.move_to_waypoint()

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def move_to_waypoint(self):
        target_x, target_y = self._waypoints[self._current_waypoint_index]
        distance = math.sqrt((target_x - self.current_position[0])**2 + (target_y - self.current_position[1])**2)

        if distance < self.tolerance:
            self.get_logger().info(f'{distance} | {self.current_position} | {(target_x, target_y)}')
            self.get_logger().info(f'Reached waypoint {self._current_waypoint_index + 1}')
            self._current_waypoint_index += 1
            if self._current_waypoint_index >= len(self._waypoints):
                self._goal_handle.succeed()
                self._goal_handle = None

                # Stopppp
                cmd_vel = Twist()
                self.publisher.publish(cmd_vel)
                self.get_logger().info(f'Finished waypoint navigation {self.current_position}')

            return

        angle_to_target = math.atan2(target_y - self.current_position[1], target_x - self.current_position[0])
        angle_diff = self.normalize_angle(angle_to_target - self.current_orientation)

        cmd_vel = Twist()

        if abs(angle_diff) > 0.1:
            # Turn in place
            cmd_vel.angular.z = 0.2 * angle_diff / abs(angle_diff)
            cmd_vel.linear.x = 0.0
        else:
            # Move forward
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.0

        self.publisher.publish(cmd_vel)

        feedback_msg = Waypoint.Feedback()
        feedback_msg.current_position = Point(x=self.current_position[0], y=self.current_position[1], z=0.0)
        feedback_msg.distance = distance
        self._goal_handle.publish_feedback(feedback_msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle
        self._waypoints = [(wp.position.x, wp.position.y) for wp in goal_handle.request.waypoints.poses]
        self._current_waypoint_index = 0

        while self._goal_handle is not None:
            rclpy.spin_once(self)
            if self._current_waypoint_index >= len(self._waypoints):
                break
        
        result = Waypoint.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()