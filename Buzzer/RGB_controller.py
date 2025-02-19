#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** ROS2 RGB LED Controller for RRC Lite                 **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- Provides control of the RGB LEDs on the RRC Lite board.
- Allows setting colors, blinking LEDs, and turning them off.
- Uses `ros_robot_controller_sdk` for low-level control.

Usage:
- Import this file and use simple commands:
    `rgb_controller.set_color(1, 255, 0, 0)`  # Set LED 1 to Red
    `rgb_controller.blink(1, 255, 255, 0, 3, 0.5)`  # Blink LED 1 in Yellow
    `rgb_controller.turn_off(1)`  # Turn off LED 1
"""

import time
import signal
import rclpy
from rclpy.node import Node
import ros_robot_controller_sdk as rrc  # RRC Lite SDK

class RGBLEDController(Node):
    """Simplified RGB LED Controller for RRC Lite"""

    def __init__(self):
        super().__init__('rgb_led_controller')
        self.board = rrc.Board()  # Initialize board
        self.get_logger().info("RGB LED Controller initialized.")
        self.turn_off_all()  # Turn off all LEDs on startup

    def set_color(self, index, red, green, blue):
        """
        Set LED color
        :param index: LED index (1 or 2)
        :param red: Red value (0-255)
        :param green: Green value (0-255)
        :param blue: Blue value (0-255)
        """
        self.board.set_rgb([[index, red, green, blue]])
        self.get_logger().info(f"Set LED {index} to R={red}, G={green}, B={blue}")

    def turn_off(self, index):
        """Turn off a specific LED."""
        self.set_color(index, 0, 0, 0)
        self.get_logger().info(f"Turned off LED {index}")

    def turn_off_all(self):
        """Turn off all LEDs."""
        self.board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])
        self.get_logger().info("All LEDs turned off")

    def blink(self, index, red, green, blue, repeat=3, delay=0.5):
        """
        Blink an LED a certain number of times.
        :param index: LED index (1 or 2)
        :param red: Red value (0-255)
        :param green: Green value (0-255)
        :param blue: Blue value (0-255)
        :param repeat: Number of times to blink
        :param delay: Time between blinks
        """
        for _ in range(repeat):
            self.set_color(index, red, green, blue)
            time.sleep(delay)
            self.turn_off(index)
            time.sleep(delay)
        self.get_logger().info(f"Blinked LED {index} {repeat} times")

def main():
    """Test the RGB LED functions"""
    rclpy.init()
    rgb_controller = RGBLEDController()

    try:
        rgb_controller.set_color(1, 255, 0, 0)  # Set LED 1 to red
        time.sleep(1)
        rgb_controller.set_color(2, 0, 255, 0)  # Set LED 2 to green
        time.sleep(1)
        rgb_controller.blink(1, 0, 0, 255, repeat=3, delay=0.5)  # Blink LED 1 in blue
        time.sleep(1)
        rgb_controller.turn_off_all()  # Turn off all LEDs

    finally:
        rgb_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
