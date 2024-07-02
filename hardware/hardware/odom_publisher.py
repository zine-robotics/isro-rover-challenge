#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

RADIUS = 0.09
BOT_BASE = 0.70

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.read_serial_data)
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        time.sleep(2)  # Allow time for the serial connection to establish

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                # Read a line of data from the serial port
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    # Split the line into individual angular velocity values
                    data = line.split(',')
                    if len(data) == 6:
                        angular_velocityR1 = float(data[0])
                        angular_velocityR2 = float(data[1])
                        angular_velocityR3 = float(data[2])
                        angular_velocityL1 = float(data[3])
                        angular_velocityL2 = float(data[4])
                        angular_velocityL3 = float(data[5])

                        avg_linear_velocityR = (angular_velocityR1 + angular_velocityR2 + angular_velocityR3) * RADIUS / 3.0
                        avg_linear_velocityL = (angular_velocityL1 + angular_velocityL2 + angular_velocityL3) * RADIUS / 3.0
                        
                        bot_linear_velocity = (avg_linear_velocityR + avg_linear_velocityL) / 2.0
                        bot_angular_velocity = (avg_linear_velocityR - avg_linear_velocityL) / BOT_BASE
                        
                      

                        # Odometry calculation
                        current_time = self.get_clock().now()
                        dt = (current_time - self.last_time).nanoseconds / 1e9

                        delta_x = bot_linear_velocity * math.cos(self.theta) * dt
                        delta_y = bot_linear_velocity * math.sin(self.theta) * dt
                        delta_theta = bot_angular_velocity * dt

                        self.x += delta_x
                        self.y += delta_y
                        self.theta += delta_theta

                        # Create Odometry message
                        odom = Odometry()
                        odom.header.stamp = current_time.to_msg()
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_link'

                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.position.z = 0.0
                        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
                        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

                        odom.twist.twist.linear.x = bot_linear_velocity
                        odom.twist.twist.angular.z = bot_angular_velocity

                        # Publish the Odometry message
                        self.odom_publisher.publish(odom)

                        # Publish the transform over tf
                        t = TransformStamped()
                        t.header.stamp = current_time.to_msg()
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_link'

                        t.transform.translation.x = self.x
                        t.transform.translation.y = self.y
                        t.transform.translation.z = 0.0
                        t.transform.rotation.z = math.sin(self.theta / 2.0)
                        t.transform.rotation.w = math.cos(self.theta / 2.0)

                        self.tf_broadcaster.sendTransform(t)

                        self.last_time = current_time

                    else:
                        print("Received data in unexpected format")

            except ValueError as e:
                self.get_logger().error(f"Error parsing data: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
