#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Omni-Wheel Motor Control in ROS2                     **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- Provides motor control for an omni-wheel robot in ROS2.
- Supports movement in all directions (forward, backward, left, right, diagonal).
- Includes rotational movement (rotate left, rotate right) and stop.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode

# Initialize ROS2
rclpy.init()
node = OmniWheelControlNode()

try:
    # Example movement commands (executed in order)
    node.move_forward(0.5, 3.0)  # Forward for 3 seconds
    node.move_backward(0.5, 3.0)  # Backward for 3 seconds
    node.move_left(0.5, 2.0)  # Left for 2 seconds
    node.move_right(0.5, 2.0)  # Right for 2 seconds
    node.move_in_direction(45, 0.5, 2.0)  # Move diagonally (45°) for 2 seconds
    node.rotate_left(0.5, 2.0)  # Rotate CCW for 2 seconds
    node.rotate_right(0.5, 2.0)  # Rotate CW for 2 seconds

finally:
    node.stop_all_motors()
    node.destroy_node()
    rclpy.shutdown()
