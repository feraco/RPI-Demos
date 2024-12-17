#!/usr/bin/env python3
# encoding: utf-8

from robot_controller_utils import MotorController, IMUDataProcessor
import rclpy

def main():
    # Initialize the ROS2 Python library
    rclpy.init()

    try:
        # Create an instance of the MotorController
        motor_controller = MotorController()
        motor_controller.get_logger().info("Moving forward for 3 seconds...")
        motor_controller.move_forward(speed=0.5)

        rclpy.spin_once(motor_controller, timeout_sec=3.0)  # Run for 3 seconds

        motor_controller.get_logger().info("Stopping motors...")
        motor_controller.stop_all_motors()

        # Process IMU data
        imu_processor = IMUDataProcessor()
        imu_processor.get_logger().info("Listening to IMU data for 3 seconds...")
        rclpy.spin_once(imu_processor, timeout_sec=3.0)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup: destroy nodes and shutdown ROS2
        motor_controller.destroy_node()
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
