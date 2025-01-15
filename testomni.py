#!/usr/bin/env python3

"""
**********************************************************
** Omni-Wheel Robot Test                                **
** Developed by: Frederick Feraco                       **
**********************************************************

Description:
- Tests all movement commands for the omni-wheel robot.
- Uses a speed of 2 for 2 seconds for each command.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode


def main(args=None):
    """
    Main function to test all movement commands.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        # Test each movement command
        node.get_logger().info('Testing forward movement...')
        node.move_forward(2, 2)  # Move forward at speed 2 for 2 seconds

        node.get_logger().info('Testing backward movement...')
        node.move_backward(2, 2)  # Move backward at speed 2 for 2 seconds

        node.get_logger().info('Testing left movement...')
        node.move_left(2, 2)  # Move left at speed 2 for 2 seconds

        node.get_logger().info('Testing right movement...')
        node.move_right(2, 2)  # Move right at speed 2 for 2 seconds

        node.get_logger().info('Testing diagonal movement (45°)...')
        node.move_in_direction(45, 2, 2)  # Move diagonally (45°) at speed 2 for 2 seconds

        node.get_logger().info('Testing rotate left (counterclockwise)...')
        node.rotate_left(2, 2)  # Rotate left (CCW) at angular speed 2 for 2 seconds

        node.get_logger().info('Testing rotate right (clockwise)...')
        node.rotate_right(2, 2)  # Rotate right (CW) at angular speed 2 for 2 seconds

        node.get_logger().info('Testing stop command...')
        node.stop_all_motors()  # Ensure the robot stops

    except KeyboardInterrupt:
        # Handle shutdown gracefully
        node.get_logger().info('Test interrupted. Stopping motors...')
        node.stop_all_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Omni-Wheel Robot Test completed.")


if __name__ == '__main__':
    main()
