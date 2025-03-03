#!/usr/bin/env python3

import cv2
import numpy as np
import socket
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
from rclpy.qos import qos_profile_sensor_data

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')

        # Initialize odometry data variables
        self.x_spot = None
        self.y_spot = None
        self.z_spot = None

        # Subscribe to the odometry topic using sensor data QoS profile
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry',  # Change this if your topic name is different (e.g., "/odom")
            self.odom_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /odometry topic.")

        # Create a timer to print a heartbeat message every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        #self.get_logger().info("Odometry callback triggered.")
        # Extract the position from the odometry message and store it
        self.x_spot = msg.pose.pose.position.x
        self.y_spot = msg.pose.pose.position.y
        self.z_spot = msg.pose.pose.position.z

        # Log the received odometry using the instance variables
        #self.get_logger().info(f"Received odometry: x={self.x_spot}, y={self.y_spot}, z={self.z_spot}")

        # Format the position into a CSV string: "x,y,z"
        data_str = f"{self.x_spot},{self.y_spot},{self.z_spot}"
        self.get_logger().debug(f"Formatted data: {data_str}")

    def timer_callback(self):
        # Heartbeat to verify the node is running
        self.get_logger().info("odom_sender node is alive.")

    def get_coords(self):
        # Create a UDP socket to listen for incoming coordinates
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"Listening on {UDP_IP}:{UDP_PORT}")

        src_points = []  # Will hold the Spot odometry points (source)
        dst_points = []  # Will hold the real-world points received via UDP

        while len(dst_points) < 5:
            data, addr = sock.recvfrom(1024)
            print('check1')
            decoded = data.decode("utf-8").strip()
            coords = decoded.split(',')
            if len(coords) == 3:
                try:
                    x, y, z = map(float, coords)
                    dst_points.append([x, y, z])
                    # Only add source point if odometry data is available
                    if self.x_spot is not None and self.y_spot is not None and self.z_spot is not None:
                        src_points.append([self.x_spot, self.y_spot, self.z_spot])
                    else:
                        print("Odometry data not yet available, skipping this point.")
                        continue
                    print(f"Got #{len(dst_points)} from {addr}: {x:.2f}, {y:.2f}, {z:.2f}")
                except ValueError:
                    print(f"Invalid numeric data: {decoded}")
            else:
                print(f"Invalid data: {decoded}")

        print("Received all points, estimating transform...")
        transform_4x4, inliers = estimate_3d_transform(src_points, dst_points)

        if transform_4x4 is not None:
            print("Transformation matrix (World -> HoloLens):\n", transform_4x4)
            print("\nInliers:\n", inliers)
        else:
            print("Failed to compute transformation.")
        
        while True:
            data, addr = sock.recvfrom(1024)
            print('check1')
            decoded = data.decode("utf-8").strip()
            coords = decoded.split(',')
            if len(coords) == 3:

                x, y, z = map(float, coords)
                coordinates = np.array([[x], [y], [z], [1]])
                inv_transform = np.linalg.inv(np.array(transform_4x4))
                result = np.matmul(inv_transform, coordinates)
                print(result)

def estimate_3d_transform(src_points, dst_points):
    # Convert the source and destination points to NumPy arrays
    src = np.array(src_points, dtype=np.float32)
    dst = np.array(dst_points, dtype=np.float32)

    retval, transform_3x4, inliers = cv2.estimateAffine3D(src, dst)
    if retval and transform_3x4 is not None:
        # Build a 4x4 transformation matrix from the 3x4 output
        full_transform = np.eye(4, dtype=np.float32)
        full_transform[:3, :4] = transform_3x4
        return full_transform, inliers
    return None, None

def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    
    # Spin briefly to allow callbacks to update odometry data
    rclpy.spin_once(node, timeout_sec=2.0)
    node.get_logger().info("Starting get_coords function.")
    
    # Call get_coords (this will block until 5 UDP points are received)
    node.get_coords()
    
    # Continue spinning if needed (or you can exit after get_coords())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("odom_sender node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
