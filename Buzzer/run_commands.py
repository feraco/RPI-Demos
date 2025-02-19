#!/usr/bin/env python3
# encoding: utf-8


import time
import rclpy
from omni_robot_controller import OmniWheelControlNode
from list_commands import run_commands  # Import command execution function
from sensor_msgs.msg import LaserScan
import lidar_conditions  # Import the LiDAR functions

def print_lidar_reading(node):
    rclpy.spin_once(node)  # Process LiDAR data
    print("LiDAR Readings:")
    print(f"Front Distance: {node.front_distance} meters")
    print(f"Back Distance: {node.back_distance} meters")
    print(f"Left Distance: {node.left_distance} meters")
    print(f"Right Distance: {node.right_distance} meters")

def main():
    rclpy.init()
    node = OmniWheelControlNode()  # Create OmniWheel control node
    
    try:
        rclpy.spin_once(node)
        run_commands(node)  # Execute commands from list_commands.py
    finally:
        node.stop_all_motors()  # Ensure motors stop before exit
        node.destroy_node()  # Properly destroy the ROS2 node
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == "__main__":
    main()
