#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from enum import Enum

# Define button mappings for PlayStation controllers
BUTTON_MAP = [
    'cross', 'circle', 'square', 'triangle', 'l1', 'r1', 'l2', 'r2',
    'select', 'start', 'l3', 'r3', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd'
]

class ButtonState(Enum):
    NORMAL = 0
    PRESSED = 1
    HOLDING = 2
    RELEASED = 3

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_control')

        # Subscribe to joystick input topic
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Store the last button states
        self.last_buttons = {button: 0 for button in BUTTON_MAP}

        # Dynamic mapping of buttons to functions
        self.button_mappings = {}

        self.get_logger().info("Joystick Controller Initialized and Ready")

    def map_button(self, button_name, function):
        """Dynamically maps a button to a function."""
        if button_name in BUTTON_MAP:
            self.button_mappings[button_name] = function
            self.get_logger().info(f"Mapped '{button_name}' to function '{function.__name__}'")
        else:
            self.get_logger().warn(f"Button '{button_name}' is not recognized.")

    def joy_callback(self, joy_msg):
        """Processes joystick button presses and executes mapped functions."""
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        for button, value in buttons.items():
            if value != self.last_buttons[button]:  # Check for button state change
                new_state = ButtonState.PRESSED if value > 0 else ButtonState.RELEASED

                if new_state == ButtonState.PRESSED and button in self.button_mappings:
                    self.get_logger().info(f"Button '{button}' pressed. Executing mapped function...")
                    self.button_mappings[button]()  # Execute assigned function

        self.last_buttons = buttons  # Update button states

def main():
    rclpy.init()
    joystick_node = JoystickController()
    rclpy.spin(joystick_node)

if __name__ == "__main__":
    main()
