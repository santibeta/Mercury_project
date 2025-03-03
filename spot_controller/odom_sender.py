#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import sys
from rclpy.qos import qos_profile_sensor_data

class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')
        
        self.get_logger().info("odom_sender node initializing...")

        # Configure remote TCP endpoint
        self.tcp_ip = '10.0.0.227'  # Replace with your remote receiver IP
        self.tcp_port = 5005        # Replace with your desired TCP port

        # Create a TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Attempt to connect to the remote TCP server
        try:
            self.sock.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info(f"TCP connection established with {self.tcp_ip}:{self.tcp_port}")
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to {self.tcp_ip}:{self.tcp_port} - {e}")
            sys.exit(1)

        # Subscribe to the odometry topic using sensor data QoS profile
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry',  # Change this if your topic is named differently (e.g., "/odom")
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /odometry topic.")

        # Create a timer to print a heartbeat message every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Odometry callback triggered.")
        # Extract the position from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Log the received odometry
        self.get_logger().info(f"Received odometry: x={x}, y={y}, z={z}")

        # Format the position into a CSV string: "x,y,z"
        data_str = f"{x},{y},{z}"
        data_bytes = data_str.encode('utf-8')

        # Send the data over TCP
        try:
            self.sock.sendall(data_bytes)
            self.get_logger().debug(f"Sent TCP data: {data_str}")
        except socket.error as e:
            self.get_logger().error(f"TCP send failed: {e}")

    def timer_callback(self):
        # Heartbeat to verify the node is running
        self.get_logger().info("odom_sender node is alive.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("odom_sender node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
