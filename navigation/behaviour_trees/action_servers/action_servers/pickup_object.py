import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from interfaces.action import Pickup 
from geometry_msgs.msg import Pose, PoseStamped


class PickupObjectActionServer(Node):
    def __init__(self):
        super().__init__('pickup_object')
        self.action_server = ActionServer(
            self,
            Pickup,
            'pickup_object',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            'object_pose',
            self.pose_callback,
            10
        )

        self.pose_status_subscription = self.create_subscription(
            PoseStamped,
            'pose_status',
            self.pose_status_callback,
            1
        )
        
        self.final_pose_publisher = self.create_publisher(
            Pose,
            'final_object_pose',
            1
        )
        self.current_position = None
        self.current_orientation = None
        self.tolerance = 0.1
        self._goal_handle = None
        self.poses = []
        self.aligned = False
        self.published_final_pose = False
        self.confirm_pick = False

        self.get_logger().info('Initialized Pick Up Object Action Server')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        if not self.aligned or self.published_final_pose:
            return
        
        if len(self.poses) < 5:
            self.poses.append(msg.pose)
        else:
            sum_x = sum(p.position.x for p in self.poses)
            sum_y = sum(p.position.y for p in self.poses)
            sum_z = sum(p.position.z for p in self.poses)
            sum_qx = sum(p.orientation.x for p in self.poses)
            sum_qy = sum(p.orientation.y for p in self.poses)
            sum_qz = sum(p.orientation.z for p in self.poses)
            sum_qw = sum(p.orientation.w for p in self.poses)

            n = len(self.poses)
            avg_pose = Pose()
            avg_pose.position.x = sum_x / n
            avg_pose.position.y = sum_y / n
            avg_pose.position.z = sum_z / n
            avg_pose.orientation.x = sum_qx / n
            avg_pose.orientation.y = sum_qy / n
            avg_pose.orientation.z = sum_qz / n
            avg_pose.orientation.w = sum_qw / n

            self.published_final_pose = True
            self.final_pose_publisher.publish(avg_pose)

    def pose_status_callback(self, msg):
        self.get_logger().info('Received object pose status')
        print(msg)

    def align_bot(self):
        self.get_logger().info('Trying to align the bot..')
        self.aligned = True

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle
        self.object_position = goal_handle.request.object_position
        feedback_msg = Pickup.Feedback()

        while self._goal_handle is not None:
            rclpy.spin_once(self)
            # Some break condition pls
        
        self.align_bot()
        self.get_final_object_pose()

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