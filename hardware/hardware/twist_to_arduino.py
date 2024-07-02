#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import serial
from rclpy.node import Node
import time

default_serial_port = '/dev/ttyUSB0'
baud_rate = 115200
reconnect_delay = 5

class TwistToArduinoNode(Node):

    def __init__(self):
        super().__init__('twist_to_arduino_node')
        self.declare_parameter('serial_port', default_serial_port)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Twist,
            'demo/cmd_vel',
            self.twist_callback,
            10
        )

        self.ser = None
        self.connect_to_serial()

    def connect_to_serial(self):
        while self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(self.serial_port, baud_rate, timeout=0.5)
                self.get_logger().info(f'Successfully connected to Arduino on port {self.serial_port}')
                time.sleep(2)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to Arduino on port {self.serial_port}: {e}")
                self.ser = None
                time.sleep(reconnect_delay)

    def twist_callback(self, msg: Twist):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Cannot send data, serial connection is not established.')
            self.connect_to_serial()
            return

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        right_linear_vel = linear_vel + (angular_vel * 0.70 / 2)
        left_linear_vel = linear_vel - (angular_vel * 0.70 / 2)

        self.get_logger().info(f"Linear velocity: {right_linear_vel}, Angular velocity: {left_linear_vel}")

        r1 = right_linear_vel * 100
        r2 = right_linear_vel * 100
        r3 = right_linear_vel * 100
        l1 = left_linear_vel * 100
        l2 = left_linear_vel * 100
        l3 = left_linear_vel * 100

        message = f"R1:{r1:.2f} R2:{r2:.2f} R3:{r3:.2f} L1:{l1:.2f} L2:{l2:.2f} L3:{l3:.2f}\n"
        try:
            self.ser.write(message.encode())
            self.get_logger().debug(f"Sent message: {message}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to Arduino: {e}")
            self.ser.close()
            self.ser = None
            self.connect_to_serial()

def main(args=None):
    rclpy.init(args=args)
    node = TwistToArduinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.ser and node.ser.is_open:
        node.ser.close()
        node.get_logger().info("Serial port closed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

