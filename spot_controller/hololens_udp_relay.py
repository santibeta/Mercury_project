#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

import socket
import threading

class HololensUDPRelay(Node):
    """
    A ROS 2 node that continuously listens on a UDP socket for data in the format "x,y,z|yaw",
    but ignores z and yaw. It only publishes Pose2D(x, y, theta=0.0) to /mv_commands.
    """

    def __init__(self, udp_host='0.0.0.0', udp_port=5005):
        super().__init__('hololens_udp_relay')

        # Create a publisher to /mv_commands (Pose2D)
        self.publisher = self.create_publisher(Pose2D, 'mv_commands', 10)

        # Create and configure the UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_host, udp_port))
        # Set a small timeout so we can periodically check for rclpy.ok() to exit
        self.sock.settimeout(0.5)

        self.get_logger().info(f'Listening for UDP packets on {udp_host}:{udp_port} ...')

        # Flag to control our listening loop
        self._running = True

        # Start a background thread that continuously reads UDP messages
        self.listen_thread = threading.Thread(target=self.udp_listen_loop, daemon=True)
        self.listen_thread.start()

    def udp_listen_loop(self):
        """
        Background thread that listens for incoming UDP messages indefinitely.
        Expects the format "x,y,z|yaw", but we ignore z and yaw for now.
        """
        while self._running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)  # Blocks until data or timeout
            except socket.timeout:
                # No data arrived in the timeout window; just loop again
                continue
            except OSError as e:
                self.get_logger().error(f'Socket error: {e}')
                break

            if not data:
                continue  # empty message, ignore

            self.get_logger().info(f'UDP from {addr}: {data}')

            try:
                # Expect "x,y,z|yaw" - we ignore yaw here
                decoded = data.decode('utf-8').strip()
                coords_part, _ = decoded.split('|')  # discard yaw
                x_str, y_str, z_str = coords_part.split(',')
                
                x_val = float(x_str)
                y_val = float(y_str)
                # z_val = float(z_str)  # we have it, but ignoring

            except Exception as e:
                self.get_logger().error(f'Failed to parse UDP data "{data}": {e}')
                continue

            # Create Pose2D from x,y and set theta=0
            pose_msg = Pose2D(x=x_val, y=y_val, theta=0.0)

            # Publish once per message
            self.get_logger().info(f'Publishing Pose2D to /mv_commands: {pose_msg}')
            self.publisher.publish(pose_msg)

        self.get_logger().info('UDP listening thread stopped.')

    def destroy_node(self):
        """
        Extend the normal destroy_node to also stop the thread and close the socket.
        """
        self._running = False
        # Close the socket so recvfrom() unblocks
        try:
            self.sock.close()
        except Exception as e:
            self.get_logger().warn(f'Error closing socket: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HololensUDPRelay(udp_host='0.0.0.0', udp_port=5005)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
