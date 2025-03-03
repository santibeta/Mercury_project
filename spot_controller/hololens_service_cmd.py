#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import threading
import subprocess

class SitStandUDPRelay(Node):
    """
    A ROS 2 node that continuously listens on a UDP socket for commands ("sit" or "stand")
    and executes a terminal command based on the message received.
    """
    def __init__(self, udp_host='0.0.0.0', udp_port=5005):
        super().__init__('sit_stand_udp_relay')
        self.udp_host = udp_host
        self.udp_port = udp_port

        # Create and bind the UDP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_host, udp_port))
        # Set a timeout to allow periodic shutdown checks.
        self.sock.settimeout(0.5)

        self.get_logger().info(f'Listening for UDP packets on {udp_host}:{udp_port} ...')
        self._running = True
        # Start a background thread for listening to UDP messages.
        self.listen_thread = threading.Thread(target=self.udp_listen_loop, daemon=True)
        self.listen_thread.start()

    def udp_listen_loop(self):
        """
        Continuously listen for UDP messages. If a message is received that is either
        "sit" or "stand", execute the corresponding service call.
        """
        while self._running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue  # No data within the timeout, try again.
            except Exception as e:
                self.get_logger().error(f'Socket error: {e}')
                break

            if not data:
                continue

            # Decode the incoming data and normalize it.
            message = data.decode('utf-8').strip().lower()
            self.get_logger().info(f"Received UDP message from {addr}: '{message}'")

            if message == "sit":
                self.execute_command("sit")
            elif message == "stand":
                self.execute_command("stand")
            else:
                self.get_logger().warn(f"Unknown command received: '{message}'")

    def execute_command(self, command):
        """
        Execute the corresponding ROS 2 service call based on the command.
        """
        if command == "sit":
            cmd_str = "ros2 service call /sit std_srvs/srv/Trigger"
        elif command == "stand":
            cmd_str = "ros2 service call /stand std_srvs/srv/Trigger"
        else:
            return

        self.get_logger().info(f"Executing command: {cmd_str}")
        try:
            result = subprocess.run(
                cmd_str,
                shell=True,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            self.get_logger().info(f"Command output: {result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command '{cmd_str}' failed with error: {e.stderr}")

    def destroy_node(self):
        """
        Stop the UDP listening loop and close the socket before shutting down.
        """
        self._running = False
        try:
            self.sock.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing socket: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SitStandUDPRelay(udp_host='0.0.0.0', udp_port=5005)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
