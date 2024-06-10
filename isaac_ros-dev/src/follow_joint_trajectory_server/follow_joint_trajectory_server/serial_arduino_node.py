import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import serial
import time

class ArduinoCommunicationNode(Node):
    def __init__(self):
        super().__init__('arduino_communication_node')

        # Initialize serial connection with retries
        self.serial_port = '/dev/ttyACM1'  # Change as needed
        self.array_length = 5
        self.baud_rate = 115200  # Change as needed
        self.ser: serial.Serial = None
        self.reconnect_delay = 5  # Delay between reconnection attempts in seconds
        self.connect_to_arduino()
        self.receive_array = []
        self.send_array = []

        # Create a subscriber to receive array data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'goal_array_topic',
            self.send_array_callback,
            10
        )

        # Create a publisher to send received array data
        self.publisher_ = self.create_publisher(Int32MultiArray, 'feedback_array_topic', 10)

        # Timer to periodically check connection
        # self.create_timer(1.0, self.check_connection)

        # Timer to publish feedback
        self.create_timer(1, self.publish_feedback)

    def connect_to_arduino(self):
        while self.ser is None:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
                self.get_logger().info('Successfully connected to Arduino')
                time.sleep(2)  # Wait for the connection to establish
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to connect to Arduino: {e}')
                self.get_logger().info(f'Retrying in {self.reconnect_delay} seconds...')
                time.sleep(self.reconnect_delay)

    def check_connection(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial connection lost. Attempting to reconnect...')
            self.ser = None
            self.connect_to_arduino()

    def send_array_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Cannot send data, serial connection is not established.')
            return

        self.send_array = msg.data
        print('self.send_array', self.send_array)
        if len(self.send_array) != self.array_length:
            self.get_logger().warn("Array must have exactly", self.array_length,"integers.")
            return

        arr_str = '<' + ','.join(map(str, self.send_array)) + '\n'

        self.ser.write(arr_str.encode())
        time.sleep(0.1)  # Wait for the Arduino to process the data
        print('debug 1')
        return

    def publish_feedback(self):
        self.receive_array = []
        i = 10  # 10 iterations to try and get the correct string

        while i:
            i -= 1
            try:
                data = self.ser.readline().decode().strip()
            except UnicodeDecodeError as e:
                self.get_logger().warn('Decoding error')
                continue
            self.get_logger().info(f"Raw received data: {data}")
            if data.startswith('<'):
                data = data[1:]
                try:
                    self.receive_array = list(map(int, data.split(',')))
                except ValueError as e:
                    self.get_logger().warn('Error parsing received data. Retrying...')
            if len(self.receive_array) == self.array_length:
                break

        if len(self.receive_array) == self.array_length:
            self.ser. reset_input_buffer()
            response_msg = Int32MultiArray()
            response_msg.data = self.receive_array
            self.publisher_.publish(response_msg)
            self.get_logger().info(f"Sent data: {self.receive_array}")

def main(args=None):
    rclpy.init(args=args)

    arduino_communication_node = ArduinoCommunicationNode()

    rclpy.spin(arduino_communication_node)

    if arduino_communication_node.ser is not None:
        arduino_communication_node.ser.close()
    arduino_communication_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()