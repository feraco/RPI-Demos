#!/usr/bin/env python3
# encoding: utf-8

from robot_controller_utils import RGBLEDController
import rclpy

def main():
    # Initialize the ROS2 Python library
    rclpy.init()

    try:
        # Create an instance of the RGBLEDController
        rgb_controller = RGBLEDController()
        rgb_controller.get_logger().info("Setting all LEDs to red...")
        rgb_controller.set_all_colors(255, 0, 0)  # Red color

        rclpy.spin_once(rgb_controller, timeout_sec=3.0)

        rgb_controller.get_logger().info("Setting all LEDs to green...")
        rgb_controller.set_all_colors(0, 255, 0)  # Green color
        rclpy.spin_once(rgb_controller, timeout_sec=3.0)

        rgb_controller.get_logger().info("Turning off all LEDs...")
        rgb_controller.turn_off_all()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup: destroy nodes and shutdown ROS2
        rgb_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
