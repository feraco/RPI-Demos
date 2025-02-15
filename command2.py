#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Robot Commands File with Joystick Integration        **
**********************************************************

This file allows mapping joystick buttons to robot commands.
"""

import rclpy
from omni_robot_controller import OmniWheelControlNode  # Ensure this matches your modufrom joystick_control import JoystickController  # Import joystick controller
from image_capture import ImageCaptureNode
from joystick_control import JoystickController  # Import joystick controller
from qr_code_tools import QRCodeDetector, generate_qr_code

def detect_qr_code():
    """Detect QR codes when a joystick button is pressed."""
    rclpy.init()
    detector = QRCodeDetector()
    rclpy.spin_once(detector)  # Process one event loop cycle
    detected_qr = detector.get_detected_qr_code()
    detector.destroy_node()
    rclpy.shutdown()
    
    if detected_qr:
        print(f"QR Code Data: {detected_qr}")
    else:
        print("No QR Code detected.")


def capture_photo():
    """ Capture an image using the camera. """
    image_node = ImageCaptureNode()  # Create an instance of ImageCaptureNode
    rclpy.spin_once(image_node)  # Process one cycle of ROS2 event loop
    image_node.capture_photo()
    image_node.destroy_node()  # Destroy the node after capturing
# Map the "R3" button to capture a photo

def cross_pressed(node):
    """Move forward when 'Cross' button is pressed"""
    node.get_logger().info("Cross button pressed. Moving forward.")
    node.move_forward(0.5, 2.0)

def square_pressed(node):
    """Move left when 'Square' button is pressed"""
    node.get_logger().info("Square button pressed. Moving left.")
    node.move_left(0.5, 2.0)

def circle_pressed(node):
    """Move right when 'Circle' button is pressed"""
    node.get_logger().info("Circle button pressed. Moving right.")
    node.move_right(0.5, 2.0)

def triangle_pressed(node):
    """Move backward when 'Triangle' button is pressed"""
    node.get_logger().info("Triangle button pressed. Moving backward.")
    node.move_backward(0.5, 2.0)

def l1_pressed(node):
    """Rotate left when 'L1' button is pressed"""
    node.get_logger().info("L1 button pressed. Rotating left.")
    node.rotate_left(0.5, 1.5)

def r1_pressed(node):
    """Rotate right when 'R1' button is pressed"""
    node.get_logger().info("R1 button pressed. Rotating right.")
    node.rotate_right(0.5, 1.5)

def main():
    rclpy.init()
    node = OmniWheelControlNode()  # Initialize the robot control node
    image_node = ImageCaptureNode()
    joystick = JoystickController()  # Initialize joystick control
    joystick.map_button("r3", capture_photo)

    # Debugging: Log joystick initialization
    node.get_logger().info("Mapping joystick buttons to robot functions...")

    # Define button mappings
    
    joystick.map_button("square", lambda: node.move_left(0.5, 2.0))
    joystick.map_button("circle", lambda: node.move_right(0.5, 2.0))
    joystick.map_button("triangle", lambda: node.move_backward(0.5, 2.0))
    joystick.map_button("l1", lambda: node.rotate_left(0.5, 1.5))
    joystick.map_button("r1", lambda: node.rotate_right(0.5, 1.5))
    joystick.map_button("x", detect_qr_code)

    # Debugging: Print mappings
    for button, function in joystick.button_mappings.items():
        node.get_logger().info(f"Button {button} mapped to {function.__name__}")

    try:
        rclpy.spin(joystick)  # Keep processing joystick events
    finally:
        node.stop_all_motors()  # Stop the robot at the end
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
