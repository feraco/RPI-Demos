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
  3. RGB LED control commands (set color, turn off)
  4. Buzzer control commands (play buzzer with frequency and duration)

Usage:
- Import this file in your main script:
    from robot_controller_utils import MotorController, IMUDataProcessor, RGBLEDController, BuzzerController
- Use the simplified commands to control motors, RGB LEDs, buzzer, and process IMU data.

**********************************************************
"""

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState, RGBState, RGBStates, BuzzerState
from sensor_msgs.msg import Imu


### Motor Controller ###
class MotorController(Node):
    """Simplified motor controller for movement commands."""
    def __init__(self):
        super().__init__('motor_controller')
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        self.get_logger().info("Motor Controller initialized and ready for commands.")

    def publish_motor_command(self, motor_states):
        msg = MotorsState()
        msg.data = motor_states
        self.motor_pub.publish(msg)

    def move_all_motors(self, speed):
        motor_states = [MotorState(id=i, rps=speed) for i in range(1, 5)]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving all motors at {speed} RPS.")

    def stop_all_motors(self):
        self.move_all_motors(0.0)
        self.get_logger().info("All motors stopped.")

    def move_forward(self, speed=0.5):
        self.move_all_motors(speed)

    def move_backward(self, speed=0.5):
        self.move_all_motors(-speed)

    def move_left(self, speed=0.5):
        motor_states = [MotorState(id=1, rps=-speed), MotorState(id=2, rps=speed),
                        MotorState(id=3, rps=-speed), MotorState(id=4, rps=speed)]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving left at {speed} RPS.")

    def move_right(self, speed=0.5):
        motor_states = [MotorState(id=1, rps=speed), MotorState(id=2, rps=-speed),
                        MotorState(id=3, rps=speed), MotorState(id=4, rps=-speed)]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f"Moving right at {speed} RPS.")


### IMU Data Processor ###
class IMUDataProcessor(Node):
    """Simplified IMU data processor."""
    def __init__(self):
        super().__init__('imu_data_processor')
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(Imu, self.imu_topic, self.listener_callback, 10)
        self.get_logger().info(f"Subscribed to IMU data on topic: {self.imu_topic}")

    def listener_callback(self, msg):
        self.display_imu_data(msg)

    def get_orientation(self, msg):
        return msg.orientation

    def get_angular_velocity(self, msg):
        return msg.angular_velocity

    def get_linear_acceleration(self, msg):
        return msg.linear_acceleration

    def display_imu_data(self, msg):
        orientation = self.get_orientation(msg)
        angular_velocity = self.get_angular_velocity(msg)
        linear_acceleration = self.get_linear_acceleration(msg)
        self.get_logger().info('--- IMU Data ---')
        self.get_logger().info(f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}')
        self.get_logger().info(f'Angular Velocity: x={angular_velocity.x:.2f}, y={angular_velocity.y:.2f}, z={angular_velocity.z:.2f}')
        self.get_logger().info(f'Linear Acceleration: x={linear_acceleration.x:.2f}, y={linear_acceleration.y:.2f}, z={linear_acceleration.z:.2f}')


### RGB LED Controller ###
class RGBLEDController(Node):
    """Simplified RGB LED controller."""
    def __init__(self):
        super().__init__('rgb_led_controller')
        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 10)
        self.get_logger().info("RGB LED Controller initialized and ready for commands.")

    def set_color(self, index, red, green, blue):
        msg = RGBStates()
        msg.states = [RGBState(index=index, red=red, green=green, blue=blue)]
        self.rgb_pub.publish(msg)
        self.get_logger().info(f"Set LED {index} to color R={red}, G={green}, B={blue}")

    def set_all_colors(self, red, green, blue):
        msg = RGBStates()
        msg.states = [RGBState(index=i, red=red, green=green, blue=blue) for i in range(1, 3)]
        self.rgb_pub.publish(msg)
        self.get_logger().info(f"Set all LEDs to color R={red}, G={green}, B={blue}")

    def turn_off_all(self):
        self.set_all_colors(0, 0, 0)
        self.get_logger().info("Turned off all LEDs.")


### Buzzer Controller ###
class BuzzerController(Node):
    """Simplified Buzzer controller."""
    def __init__(self):
        super().__init__('buzzer_controller')
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.get_logger().info("Buzzer Controller initialized and ready for commands.")

    def play_buzzer(self, freq=1000, on_time=0.5, off_time=0.5, repeat=1):
        """
        Play the buzzer with specified frequency and duration.
        :param freq: Frequency in Hz.
        :param on_time: Buzzer on duration in seconds.
        :param off_time: Buzzer off duration in seconds.
        :param repeat: Number of times to repeat the buzzer cycle.
        """
        msg = BuzzerState()
        msg.freq = freq
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        self.buzzer_pub.publish(msg)
        self.get_logger().info(f"Playing buzzer: freq={freq}Hz, on_time={on_time}s, off_time={off_time}s, repeat={repeat}")

    def stop_buzzer(self):
        """Stop the buzzer immediately."""
        self.play_buzzer(freq=0, on_time=0, off_time=0, repeat=1)
        self.get_logger().info("Stopped the buzzer.")


def main():
    """Test all controllers."""
    rclpy.init()
    motor_controller = MotorController()
    rgb_controller = RGBLEDController()
    buzzer_controller = BuzzerController()

    motor_controller.move_forward(0.5)
    rgb_controller.set_all_colors(255, 0, 0)
    buzzer_controller.play_buzzer(freq=1000, on_time=0.5, off_time=0.5, repeat=2)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
