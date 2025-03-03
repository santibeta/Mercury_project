import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

# UDP parameters
UDP_IP = '0.0.0.0'
UDP_PORT = 5007

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        # Publisher for cmd_vel topic with a queue size of 10.
        self.pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer callback at 10 Hz.
        self.timer = self.create_timer(0.1, self.commandCallback)
        # Default command is "stop"
        self.cmd = "stop"
        # Record the last time a command was received (in seconds)
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9
        # Timeout after which the command resets to "stop" (in seconds)
        self.command_timeout = 0.2

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.1)  # Non-blocking behavior.
        self.get_logger().info(f"UDP socket bound to {UDP_IP}:{UDP_PORT}")

        # Mapping string commands to movement parameters suitable for a four-legged robot.
        self.command_mapping = {
            "forward":         {"linear": 0.3,  "angular": 0.0},
            "backward":        {"linear": -0.3, "angular": 0.0},
            "left":            {"linear": 0.0,  "angular": 0.3},
            "right":           {"linear": 0.0,  "angular": -0.3},
            "forward_left":    {"linear": 0.3,  "angular": 0.3},
            "forward_right":   {"linear": 0.3,  "angular": -0.3},
            "backward_left":   {"linear": -0.3, "angular": 0.3},
            "backward_right":  {"linear": -0.3, "angular": -0.3},
            "stop":            {"linear": 0.0,  "angular": 0.0}
        }

    def commandCallback(self):
        twist = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds

        try:
            # Receive data from the UDP socket.
            # NOTE: Here we assume that the received data is already a string.
            data, addr = self.sock.recvfrom(1024)
            received_command = data.decode('utf-8').strip().lower()
            self.get_logger().info(f"Received command '{received_command}' from {addr}")
            # Update the last received time and store the new command.
            self.last_cmd_time = current_time
            self.cmd = received_command
        except socket.timeout:
            # If no new command is received, check if the timeout has been exceeded.
            if (current_time - self.last_cmd_time) > self.command_timeout:
                if self.cmd != "stop":
                    self.get_logger().info("No command received within timeout. Switching to 'stop'.")
                self.cmd = "stop"

        # Look up the command in the mapping dictionary.
        if self.cmd in self.command_mapping:
            mapping = self.command_mapping[self.cmd]
            twist.linear.x = mapping["linear"]
            twist.angular.z = mapping["angular"]
        else:
            self.get_logger().warn(f"Unknown command: '{self.cmd}'. Stopping robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the Twist message.
        self.pub_.publish(twist)
        self.get_logger().info(
            f"Published command '{self.cmd}' with linear.x: {twist.linear.x} and angular.z: {twist.angular.z}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
