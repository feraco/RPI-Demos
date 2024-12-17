#!/usr/bin/env python3
# encoding: utf-8

from robot_controller_utils import BuzzerController
import rclpy

def main():
    # Initialize the ROS2 Python library
    rclpy.init()

    try:
        # Create an instance of the BuzzerController
        buzzer_controller = BuzzerController()
        buzzer_controller.get_logger().info("Playing buzzer at 1000Hz for 3 cycles...")
        buzzer_controller.play_buzzer(freq=1000, on_time=0.5, off_time=0.5, repeat=3)

        rclpy.spin_once(buzzer_controller, timeout_sec=3.0)

        buzzer_controller.get_logger().info("Stopping buzzer...")
        buzzer_controller.stop_buzzer()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup: destroy nodes and shutdown ROS2
        buzzer_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
