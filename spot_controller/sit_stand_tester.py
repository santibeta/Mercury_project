#!/usr/bin/env python3

import socket
import sys

def send_udp_message(message, host='10.0.0.83', port=5010):
    """
    Sends a UDP message to the specified host and port.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        print(f"Sending '{message}' to {host}:{port}")
        sock.sendto(message.encode('utf-8'), (host, port))
    finally:
        sock.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: udp_test_client.py <sit|stand>")
        sys.exit(1)

    command = sys.argv[1].strip().lower()
    if command not in ['sit', 'stand']:
        print("Error: command must be either 'sit' or 'stand'.")
        sys.exit(1)

    # Change host if your UDP server is not on localhost.
    host = '10.0.0.83'
    port = 5010

    send_udp_message(command, host, port)

if __name__ == '__main__':
    main()
