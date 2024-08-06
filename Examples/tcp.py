import socket
import time
import subprocess
import threading

def send_data(ip_address, send_port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if sock.fileno() < 0:
        print("Failed to create socket.")
        return

    # Set server address
    server_address = (ip_address, send_port)

    # Connect to server
    while True:
        try:
            sock.connect(server_address)
            break
        except socket.error as e:
            print("Failed to connect to server. Retrying...")
            time.sleep(0.1)
            continue

    # Send data
    try:
        sock.sendall(message.encode())
    except socket.error as e:
        print("Failed to send data.")
    
    sock.close()

def receive_data():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if server_socket.fileno() < 0:
        print("Failed to create socket.")
        return ""

    # Set server address
    server_address = ('', 9088)

    # Bind server address and port
    try:
        server_socket.bind(server_address)
    except socket.error as e:
        print("Failed to bind socket.")
        server_socket.close()
        return ""

    # Listen for connection requests
    server_socket.listen(1)
    print("Server is listening for connections...")

    while True:
        # Accept client connection
        client_socket, client_address = server_socket.accept()
        if client_socket.fileno() < 0:
            print("Failed to accept connection.")
            continue

        # Receive data
        buffer_size = 1024
        buffer = client_socket.recv(buffer_size).decode()
        
        if not buffer:
            print("Failed to receive data.")
        else:
            print("Received data:", buffer)
            if buffer.strip() == "arm.make_coffee()":
                print("Executing make.py script...")
                send_data("192.168.31.8", 8028, "start")
                # Parameters for make.py
                param1 = "192.168.2.100"
                param2 = "192.168.2.108"
                param3 = "20"
                subprocess.run(["python", "make.py", param1, param2, param3])
                # Send confirmation back to client using send_data
                send_data("192.168.31.8", 8028, "true")

        #client_socket.close()

def start_receiving():
    receive_thread = threading.Thread(target=receive_data)
    receive_thread.daemon = True
    receive_thread.start()

if __name__ == "__main__":
    start_receiving()
    # Main thread can continue running other code
    while True:
        time.sleep(1)
