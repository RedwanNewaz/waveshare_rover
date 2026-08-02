#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import json
import requests
import numpy as np
import sys


class TwistToJsonNode(Node):
    def __init__(self):
        super().__init__('twist_to_json_node')
        self.v_r = 0.0  # Right wheel velocity
        self.v_l = 0.0  # Left wheel velocity
        self.angular_z = 0.0  # Angular velocity
        self.linear_x = 0.0  # Linear velocity

        # IP address for HTTP requests, settable via the 'ip_address' ROS2
        # parameter (e.g. from the launch file) instead of being hardcoded.
        self.declare_parameter('ip_address', '192.168.10.148')
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.wheel_base = 0.15 / 2.0   # Example wheel base in meters
        self.new_data_received = False
        
        # Create subscriber for cmd_vel topic
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        
        
        # Create publisher for JSON string (optional - for debugging/logging)
        self.json_publisher = self.create_publisher(
            String,
            'json_cmd',
            10
        )
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Twist to JSON node started')
        if self.ip_address:
            self.get_logger().info(f'HTTP requests will be sent to: {self.ip_address}')
        else:
            self.get_logger().info('No IP address provided - only publishing JSON messages')

    def unicycle_to_diff(self, v, omega):
        """
        Convert unicycle model (v, omega) to differential drive wheel velocities.

        Parameters:
            v (float): Linear velocity in m/s
            omega (float): Angular velocity in rad/s
            wheel_base (float): Distance between the two wheels in meters

        Returns:
            tuple: (v_left, v_right) in m/s
        """
        v_r = v + (self.wheel_base / 2.0) * omega
        v_l = v - (self.wheel_base / 2.0) * omega
        return v_l, v_r
    
    def timer_callback(self):
        """Timer callback for periodic tasks (if needed)"""
        # This can be used for periodic logging or other tasks
        if not self.new_data_received:
        # Skip sending if there's no new data
            return

        # Create JSON command in the specified format
        json_cmd = {
            "T": 1,  # Fixed value as shown in example
            "L": self.v_l,  # Linear velocity in m/s
            "R": self.v_r  # Angular velocity in rad/s
        }

        # json_cmd = {
        #     "T": 13,  # Fixed value as shown in example
        #     "X": self.linear_x,  # Linear velocity in m/s
        #     "Z": self.angular_z  # Angular velocity in rad/s
        # }
        
        # Convert to JSON string
        json_string = json.dumps(json_cmd, separators=(',', ':'))
        
        # Log the JSON command
        self.get_logger().info(f'JSON Command: {json_string}')
        
        # Publish JSON string to ROS topic
        json_msg = String()
        json_msg.data = json_string
        self.json_publisher.publish(json_msg)
        
        # Send HTTP request if IP address is provided
        if self.ip_address:
            self.send_http_request(json_string)
        
        # reset velocities for next iteration
        self.new_data_received = False
        self.v_r = 0.0
        self.v_l = 0.0
        # reset angular_z and linear_x
        self.angular_z = 0.0
        self.linear_x = 0.0
    
    def sigmoid (self, x):
        """Sigmoid function for smooth control"""
        return 1 / (1 + np.exp(-x))

    def twist_callback(self, msg):
        """Callback function for Twist messages"""
        # Extract linear.x and angular.z from Twist message

        # print(msg)
        
        # self.angular_z = -msg.angular.z
        # self.linear_x = 0.5 if abs(self.angular_z) > 0 else msg.linear.x 



        
        
        if abs(msg.linear.x) > 0.0:

            self.v_r, self.v_l = self.unicycle_to_diff(msg.linear.x, msg.angular.z)
        elif abs(msg.angular.z)> 0.0:
            self.v_r, self.v_l = msg.angular.z, -msg.angular.z
        
   
        self.new_data_received = True
     
        
        

    def send_http_request(self, json_cmd):
        """Send HTTP GET request with JSON command"""
        try:
            url = f"http://{self.ip_address}/js?json={json_cmd}"
            response = requests.get(url, timeout=10)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'HTTP request failed: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')


def main():
    # Initialize ROS2
    rclpy.init(args=sys.argv)

    # Create and run the node ('ip_address' is a ROS2 parameter -- set it via
    # the launch file's `ip_address` launch argument or `--ros-args -p ip_address:=...`)
    node = TwistToJsonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
