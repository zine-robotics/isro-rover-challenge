import socket
import threading
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

class SocketServerNode(Node):

    def __init__(self):
        super().__init__('socket_server_node')
        self.host = 'localhost'
        self.port = 8000 
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self.get_logger().info(f"Server listening on {self.host}:{self.port}")

        self.pub = self.create_publisher(String, 'base_station_message', 10)

        self.client_thread = threading.Thread(target=self.handle_clients)
        self.client_thread.start()

    def handle_clients(self):
        while True:
            client_socket, client_address = self.server_socket.accept()
            self.get_logger().info(f"Client connected: {client_address}")

            client_thread = threading.Thread(target=self.handle_client, args=(client_socket, client_address))
            client_thread.start()

    def handle_client(self, client_socket, client_address):
        chunks = []

        while True:
            try:
                chunk = client_socket.recv(1024).decode()
                if not chunk:
                    break
                chunks.append(chunk)

                if '\n' in chunk:
                    chunks[-1] = chunk[:chunk.index('\n')]
                    break

            except (socket.error, ConnectionResetError) as e:
                self.get_logger().error(f"Client connection error: {e}")
                client_socket.close()
                break
        
        data = ''.join(chunks)
        print('Received data:')

        try:
            json_data = json.loads(data)
            ros_msg = String()
            ros_msg.data = data
            self.pub.publish(ros_msg)
        except json.JSONDecodeError as e:
            print('Failed to decode JSON:', e)

        client_socket.close()
        self.get_logger().info(f"Client disconnected: {client_address}")

def main(args=None):
    rclpy.init(args=args)
    node = SocketServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
