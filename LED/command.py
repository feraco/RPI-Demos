#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Robot Commands File with LiDAR Integration           **
**********************************************************

This file contains pre-defined movement commands for the robot.
It uses LiDAR data to ensure obstacle avoidance before executing movements
and logs distance readings for each direction.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode  # Ensure this matches your node's module name

def main():
    rclpy.init()
    node = OmniWheelControlNode()  # Create the OmniWheel control node

    try:
        # Log distance data before executing movements
        node.get_logger().info(f"Front Distance: {node.front_distance:.2f} meters")
        node.get_logger().info(f"Back Distance: {node.back_distance:.2f} meters")
        node.get_logger().info(f"Left Distance: {node.left_distance:.2f} meters")
        node.get_logger().info(f"Right Distance: {node.right_distance:.2f} meters")
        node.set_all_colors(255, 0, 0)
        node.play_buzzer(1000, 2.0, 2.0, 2)
        # Check for obstacles and execute movements
        if not node.is_obstacle_near('front'):
            node.get_logger().info("No obstacle detected in front. Moving forward.")
            node.move_forward(0.5, 3.0)  # Move forward for 3 seconds
        else:
            node.get_logger().info("Obstacle detected in front. Stopping movement.")

        if not node.is_obstacle_near('back'):
            node.get_logger().info("No obstacle detected in back. Moving backward.")
            node.move_backward(0.5, 3.0)  # Move backward for 3 seconds
        else:
            node.get_logger().info("Obstacle detected in back. Stopping movement.")

        if not node.is_obstacle_near('left'):
            node.get_logger().info("No obstacle detected on the left. Moving left.")
            node.move_left(0.5, 2.0)  # Move left for 2 seconds
        else:
            node.get_logger().info("Obstacle detected on the left. Stopping movement.")

        if not node.is_obstacle_near('right'):
            node.get_logger().info("No obstacle detected on the right. Moving right.")
            node.move_right(speed=0.5, duration=2.0)  # Move right for 2 seconds
        else:
            node.get_logger().info("Obstacle detected on the right. Stopping movement.")

        # Rotations do not check for obstacles since they rotate in place
        node.rotate_left(angular_speed=0.5, duration=2.0)  # Rotate left for 2 seconds
        node.rotate_right(angular_speed=0.5, duration=2.0)  # Rotate right for 2 seconds

    finally:
        node.stop_all_motors()  # Stop the robot at the end
        node.destroy_node()  # Destroy the ROS2 node
        rclpy.shutdown()  # Shutdown ROS2

if __name__ == "__main__":
    main()
