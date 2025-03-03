#!/usr/bin/env python3

import socket

def main():
    UDP_IP = "0.0.0.0"   # Listen on all available network interfaces
    UDP_PORT = 5005      # Match the port used in odom_sender

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"odom_tester listening on {UDP_IP}:{UDP_PORT}...")

    try:
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size of 1KB
            message = data.decode('utf-8')
            print(f"Received from {addr}: {message}")
    except KeyboardInterrupt:
        print("\nodom_tester interrupted by user.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
