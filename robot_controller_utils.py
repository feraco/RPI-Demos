#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** ROS2 Robot Controller Utilities                     **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- This file serves as a reusable library for simplified commands.
- Includes:
  1. Motor control commands (forward, backward, left, right, stop)
  2. IMU data extraction commands (orientation, angular velocity, etc.)

Usage:
- Import this file in your main script:
    from robot_controller_utils import MotorController, IMUDataProcessor
- Use the simplified commands to control motors and process IMU data.

**********************************************************
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ros_robot_controller_msgs.msg import MotorsState, MotorState


class MotorController(Node):
    """
    Simplified motor controller class with easy-to-use commands
    for forward, backward, left, right movements and stopping.
    """
    def __init__(self):
        super().__init__('motor_controller')

        # Publisher for motor control
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)

        self.get_logger().info("Motor Controller initialized and ready for commands.")

    def publish_motor_command(self, motor_states):
        """Publish motor states."""
        msg = MotorsState()
        msg.data = motor_states
        self.motor_pub.publish(msg)

    def move_all_motors(self, speed):
        """Move all motors at a specified speed."""
        motor_states = [MotorState(id=i, rps=speed) for i in range(1, 5)]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving all motors at {speed} RPS.")

    def stop_all_motors(self):
        """Stop all motors."""
        self.move_all_motors(0.0)
        self.get_logger().info("All motors stopped.")

    def move_forward(self, speed):
        """Move all motors forward."""
        self.move_all_motors(speed)

    def move_backward(self, speed):
        """Move all motors backward."""
        self.move_all_motors(-speed)

    def move_left(self, speed):
        """Move left by controlling motor directions."""
        motor_states = [
            MotorState(id=1, rps=-speed),  # Left side reverse
            MotorState(id=2, rps=speed),   # Right side forward
            MotorState(id=3, rps=-speed),
            MotorState(id=4, rps=speed)
        ]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving left at {speed} RPS.")

    def move_right(self, speed):
        """Move right by controlling motor directions."""
        motor_states = [
            MotorState(id=1, rps=speed),   # Left side forward
            MotorState(id=2, rps=-speed),  # Right side reverse
            MotorState(id=3, rps=speed),
            MotorState(id=4, rps=-speed)
        ]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving right at {speed} RPS.")


class IMUDataProcessor(Node):
    """
    Simplified IMU data processor class that extracts IMU values
    using helper methods.
    """
    def __init__(self):
        super().__init__('imu_data_processor')

        # IMU topic parameter
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # IMU subscription
        self.subscription = self.create_subscription(
            Imu, self.imu_topic, self.listener_callback, 10
        )
        self.get_logger().info(f"Subscribed to IMU data on topic: {self.imu_topic}")

    def listener_callback(self, msg):
        """Callback to process incoming IMU data."""
        self.display_imu_data(msg)

    def get_orientation(self, msg):
        """Extract orientation data from the IMU message."""
        return msg.orientation

    def get_angular_velocity(self, msg):
        """Extract angular velocity data from the IMU message."""
        return msg.angular_velocity

    def get_linear_acceleration(self, msg):
        """Extract linear acceleration data from the IMU message."""
        return msg.linear_acceleration

    def display_imu_data(self, msg):
        """Display all IMU data in an organized format."""
        orientation = self.get_orientation(msg)
        angular_velocity = self.get_angular_velocity(msg)
        linear_acceleration = self.get_linear_acceleration(msg)

        self.get_logger().info('--- IMU Data ---')
        self.get_logger().info(f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}')
        self.get_logger().info(f'Angular Velocity: x={angular_velocity.x:.2f}, y={angular_velocity.y:.2f}, z={angular_velocity.z:.2f}')
        self.get_logger().info(f'Linear Acceleration: x={linear_acceleration.x:.2f}, y={linear_acceleration.y:.2f}, z={linear_acceleration.z:.2f}')


def main():
    """
    Demonstration of simplified commands.
    This can be removed when used as a library.
    """
    rclpy.init()

    # Example of motor control
    motor_controller = MotorController()
    motor_controller.move_forward(0.5)
    motor_controller.stop_all_motors()

    # Example of IMU data processing
    imu_processor = IMUDataProcessor()

    try:
        rclpy.spin(motor_controller)
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        motor_controller.get_logger().info("Shutting down controllers.")
    finally:
        motor_controller.destroy_node()
        imu_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
