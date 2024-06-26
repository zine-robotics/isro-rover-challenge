#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import serial
from rclpy.node import Node
import time

default_serial_port = '/dev/ttyACM0'  # Default port if not specified
baud_rate = 115200  # Change as needed
reconnect_delay = 5

class TwistToArduinoNode(Node):

    def __init__(self):
        super().__init__('twist_to_arduino_node')
        self.declare_parameter('serial_port', default_serial_port)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )

        # Serial port configuration
        self.ser = None
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.serial_port, baud_rate, timeout=0.5)
                self.get_logger().info(f'Successfully connected to Arduino on port {self.serial_port}')
                time.sleep(2)  # Wait for the connection to establish
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to Arduino on port {self.serial_port}: {e}")
                time.sleep(reconnect_delay)

        self.get_logger().info('TwistToArduinoNode is started.')

    def twist_callback(self, msg: Twist):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Cannot send data, serial connection is not established.')
            return

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        right_linear_vel = linear_vel + (angular_vel*0.70/2)
        left_linear_vel =linear_vel - (angular_vel*0.70/2) # Apply the same scaling if needed

        self.get_logger().info(f"Linear velocity: {right_linear_vel}, Angular velocity: {left_linear_vel}")

        # Construct the message to send
        # Assuming mapping to the motors is as follows:
        # R1, R2, R3, L1, L2, L3 are the individual motor targets
        r1 = right_linear_vel
        r2 = right_linear_vel
        r3 = right_linear_vel
        l1 = left_linear_vel
        l2 = left_linear_vel
        l3 = left_linear_vel

        message = f"R1:{r1:.2f} R2:{r2:.2f} R3:{r3:.2f} L1:{l1:.2f} L2:{l2:.2f} L3:{l3:.2f}\n"
        self.ser.write(message.encode())

        self.get_logger().debug(f"Sent message: {message}")

def main(args=None):
    rclpy.init(args=args)
    node = TwistToArduinoNode()

    rclpy.spin(node)

    # Close the serial port before shutting down the node
    if node.ser.is_open:
        node.ser.close()
        node.get_logger().info("Serial port closed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
