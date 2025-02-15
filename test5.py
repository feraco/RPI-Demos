#!/usr/bin/env python3

"""
**********************************************************
** Omni-Wheel Robot - 45Â° Movement Test with Return     **
** Moves forward to a direction, stops, and returns.    **
**********************************************************

Description:
- Moves in 45Â° increments (0Â°, 45Â°, 90Â°, ..., 315Â°).
- Moves forward for 1 second at 1 m/s, stops, and moves back.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode
import time


def main(args=None):
    """
    Main function to test movement in 45Â° increments with return to center.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        # Speed and duration parameters
        speed = 1.0  # Speed in m/s
        duration = 1.0  # Movement duration in seconds

        # Loop through 45Â° increments (0Â°, 45Â°, ..., 315Â°)
        for degrees in range(0, 360, 45):
            # Step 1: Move forward to the specified angle
            node.get_logger().info(f'Moving forward to {degrees}Â°...')
            node.move_in_direction(degrees, speed, duration)

            # Step 2: Stop at the angle
            node.get_logger().info(f'Stopping at {degrees}Â°...')
            node.stop_all_motors()
            time.sleep(1)  # Pause for 1 second

            # Step 3: Move back to center (180Â° opposite the angle)
            return_angle = (degrees + 180) % 360  # Opposite direction
            node.get_logger().info(f'Moving backward to center at {return_angle}Â°...')
            node.move_in_direction(return_angle, speed, duration)

            # Step 4: Stop at the center
            node.get_logger().info(f'Stopping at center after {return_angle}Â°...')
            node.stop_all_motors()
            time.sleep(1)  # Pause for 1 second before the next movement

        node.get_logger().info('Completed all 45Â° movements with return to center.')

    except KeyboardInterrupt:
        # Gracefully handle user interruption
        node.get_logger().info('Test interrupted. Stopping motors...')
        node.stop_all_motors()

    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        print("Omni-Wheel Robot 45Â° Movement Test completed.")

if __name__ == '__main__':
    main()
