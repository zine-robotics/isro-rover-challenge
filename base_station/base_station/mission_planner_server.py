import socket
import json

# Define the server's IP address and port
HOST = 'localhost'
PORT = 8000

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the IP address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections
server_socket.listen(1)
print('Server listening on {}:{}'.format(HOST, PORT))

try:
    while True:
        # Accept a client connection
        client_socket, addr = server_socket.accept()
        print('Connected by', addr)

        # Initialize an empty list to collect data chunks
        chunks = []

        while True:
            # Receive data from the client
            chunk = client_socket.recv(1024).decode()
            if not chunk:
                # No more data, break the loop
                break
            chunks.append(chunk)
            # Check if the end-of-message marker is in the last chunk
            if '\n' in chunk:
                # Remove the end-of-message marker and exit the loop
                chunks[-1] = chunk[:chunk.index('\n')]
                break

        # Join all chunks to get the complete data
        data = ''.join(chunks)
        print('Received data:', data)

        # Here you can parse the JSON data
        # Ensure to handle parsing errors with try-except block
        try:
            json_data = json.loads(data)
            print('JSON data:', json_data)
        except json.JSONDecodeError as e:
            print('Failed to decode JSON:', e)

except KeyboardInterrupt:
    print('Program interrupted, closing server socket')
    server_socket.close()