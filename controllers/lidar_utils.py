#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarFunctions(Node):
    def __init__(self):
        super().__init__('lidar_functions')
        self.subscription = self.create_subscription(
            LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.lidar_data = None  # Stores latest LiDAR data

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges

    def get_lidar_distances(self):
        """Returns the front, left, right, and back distances from the LiDAR sensor."""
        if not self.lidar_data:
            return None
        
        num_points = len(self.lidar_data)
        front = self.lidar_data[num_points // 2]  # Middle = Front
        left = self.lidar_data[int(num_points * 0.75)]  # Left side
        right = self.lidar_data[int(num_points * 0.25)]  # Right side
        back = self.lidar_data[0]  # Back (assumes 360° LiDAR)
        
        return {'front': front, 'left': left, 'right': right, 'back': back}

    def is_path_clear(self, threshold=0.5):
        """Checks if there are any obstacles within the given threshold distance."""
        distances = self.get_lidar_distances()
        if not distances:
            return None

        clear = all(dist > threshold for dist in distances.values())
        return clear

    def get_closest_obstacle(self):
        """Returns the direction of the closest obstacle."""
        distances = self.get_lidar_distances()
        if not distances:
            return None
        
        closest_direction = min(distances, key=distances.get)
        return closest_direction, distances[closest_direction]

    def avoid_obstacles(self):
        """Prints navigation suggestions based on detected obstacles."""
        distances = self.get_lidar_distances()
        if not distances:
            print("⚠️ No valid LiDAR data received!")
            return
        
        if distances['front'] < 0.5:
            print("🚨 Obstacle in front! Turning left or right recommended.")
        elif distances['left'] < 0.5:
            print("⚠️ Left side blocked! Move forward or turn right.")
        elif distances['right'] < 0.5:
            print("⚠️ Right side blocked! Move forward or turn left.")
        elif distances['back'] < 0.5:
            print("🚨 Obstacle behind! Moving forward recommended.")
        else:
            print("✅ All clear! Safe to proceed.")

# Initialize node for testing in Jupyter Notebook
def main():
    rclpy.init()
    lidar_node = LidarFunctions()
    rclpy.spin_once(lidar_node)  # Fetch LiDAR data once
    print(lidar_node.get_lidar_distances())
    lidar_node.avoid_obstacles()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
