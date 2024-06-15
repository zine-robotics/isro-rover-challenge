import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import math

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')

        self.create_subscription(Int32MultiArray, 'feedback_array_topic', self.feedback_callback, 10)

        # Define your joint names
     
        # Create a JointState message object
        self.joint_state = JointState()
        self.joint_state.name = [
            'joint_0',
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5'
        ]

        # Set initial or dynamic joint positions
        self.joint_state.position = [0.0,0.0,0.0,0.0,0.0,0.0]    # Example joint positions in radians
        self.joint_state.velocity=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.joint_state.effort=[float('nan'),float('nan'),float('nan'),float('nan'),float('nan'),float('nan'),float('nan')]

        # Set the publishing rate (optional, adjust as needed)
        self.pub_rate = 10  # Hz


        # Create a publisher for joint states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Create a timer to trigger publishing at the desired rate
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)

    def feedback_callback(self, msg):
        # Use the received data as the position for joint_state
        self.joint_state.position = [math.radians(x) for x in msg.data]

    def timer_callback(self):
    
        self.joint_state.header.stamp=   self.get_clock().now().to_msg()
        # self.get_logger().info(f'Publishing joint states  {self.joint_state.position}')
        self.pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
