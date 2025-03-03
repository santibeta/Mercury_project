import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5007

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# List of test commands
commands = [
    "forward",
    "forward_left",
    "left",
    "backward_left",
    "backward",
    "backward_right",
    "right",
    "forward_right",
    "stop"
]

# Send each command with a one-second interval
for command in commands:
    sock.sendto(command.encode('utf-8'), (UDP_IP, UDP_PORT))
    print(f"Sent command: {command}")
    time.sleep(1)
