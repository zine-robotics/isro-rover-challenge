import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from interfaces.action import Pickup
from geometry_msgs.msg import PoseStamped, Twist, Pose
from std_msgs.msg import Bool
import time
import numpy as np 

class PickupObjectActionServer(Node):
    def __init__(self):
        super().__init__('pickup_object')
        
        # Action Server
        self.action_server = ActionServer(
            self,
            Pickup,
            'pickup_object',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscriptions
        self.pose_subscription = self.create_subscription(
            Pose,
            'object_pose',
            self.pose_callback,
            10
        )
        
        self.pose_status_subscription = self.create_subscription(
            Bool,
            'pose_status',
            self.pose_status_callback,
            1
        )

        # Publisher for final object pose
        self.final_pose_publisher = self.create_publisher(
            Pose,
            'final_pick_pose',
            1
        )

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'stage2/cmd_vel',
            10
        )

        # Initialize variables
        self.current_object_pose = None
        self.aligned = False
        self.published_final_pose = False
        self._goal_handle = None
        self.pickup_completed = False

        self.get_logger().info('Initialized Pick Up Object Action Server')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        self.get_logger().info(f"{msg.position}")
        self.current_object_pose = msg

    def pose_status_callback(self, msg):
        self.get_logger().info('Received object pose status')
        self.pickup_completed = msg.data

    def get_final_object_pose(self):
        obj_pose_arr = []
        for i in range(50):
            obj_pose_arr.append((self.current_object_pose.position.x,self.current_object_pose.position.y,self.current_object_pose.position.z))
            time.sleep(0.1)
        obj_pose_arr = np.array(obj_pose_arr)
        avg_position = np.mean(obj_pose_arr, axis=0)
        
        final_pose = Pose()
        final_pose.position.x = avg_position[0]
        final_pose.position.y = avg_position[1]
        final_pose.position.z = avg_position[2]

        self.final_pose_publisher.publish(final_pose)
        self.get_logger().info(f'FINAL POSE {final_pose} PUBLISHED')

    def align_bot(self):
        self.get_logger().info('Trying to align the bot...')
        
        if self.current_object_pose is None:
            self.get_logger().warning('No object pose received yet!')
            return

        # Desired ranges for x and y coordinates
        x_min, x_max = 0.6, 0.8
        y_min, y_max = -0.2, 0.2

        x = self.current_object_pose.position.x
        y = self.current_object_pose.position.y

        vel_msg = Twist()

        if x < x_min or x > x_max:
            # Adjust linear velocity to bring the object within the x range
            if x < x_min:
                vel_msg.linear.x = 0.1  # Move forward
            elif x > x_max:
                vel_msg.linear.x = -0.1  # Move backward

        if y < y_min or y > y_max:
            # Adjust angular velocity to center the object in y direction
            if y < y_min:
                vel_msg.angular.z = 0.1  # Turn left
            elif y > y_max:
                vel_msg.angular.z = -0.1  # Turn right

        # If no adjustments are needed, stop the robot
        if not (x < x_min or x > x_max or y < y_min or y > y_max):
            self.get_logger().info('Object is within the desired range.')
            self.aligned = True
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(vel_msg)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle
        self.object_position = goal_handle.request.object_position

        # implementing align bot
        # while not self.aligned:
        #     rclpy.spin_once(self)
        #     self.align_bot()

        self.get_logger().info('Bot aligned successfully.')
        
        self.get_final_object_pose()
        
        while(1):
            if self.pickup_completed == True:
                break

        goal_handle.succeed()
        
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
