


#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Omni-Wheel Motor Control in ROS2                     **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- Provides motor control for an omni-wheel robot in ROS2.
- Supports movement in all directions (forward, backward, left, right, diagonal).
- Includes rotational movement (rotate left, rotate right) and stop.
"""

import rclpy  # ROS2 Python library
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState, RGBState, RGBStates, BuzzerState
from sensor_msgs.msg import LaserScan
import math
import time
from mecanum import MecanumChassis  # Import the MecanumChassis class

class OmniWheelControlNode(Node):
    def __init__(self):
        super().__init__('omni_wheel_control_node')  # Initialize the node
        self.robot_moving = False  # Track movement state

        self.min_distance = float('inf')
        self.avg_distance = float('inf')
        # Initialize MecanumChassis with default parameters
        self.mecanum = MecanumChassis(wheelbase=0.1368, track_width=0.1446, wheel_diameter=0.065)

        # Publisher for motor commands
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)

        # Publisher for Buzzer
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)

        # Publisher for RGB LEDs
        self.rgb_pub = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 10)

        # Subscriber for LiDAR data
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)

        self.front_distance = float('inf')
        self.back_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.critical_distance = 0.1  # Set critical distance to 10 cm
        self.get_logger().info('Omni-Wheel Control Node initialized and ready for commands.')

    def process_lidar_data(self, msg):
        """Processes LiDAR data and finds the minimum and average distances."""
        self.get_logger().info("LiDAR callback triggered!")  # Debugging message

        ranges = msg.ranges
        num_readings = len(ranges)

        if num_readings == 0:
            self.get_logger().warn("LiDAR message received, but ranges list is empty!")
            return

        # Remove 'inf' values and negative values
        valid_ranges = [r for r in ranges if 0.01 < r < float('inf')]

        if not valid_ranges:
            self.get_logger().warn("No valid LiDAR data available (all inf or out of range)")
            self.min_distance = float('inf')
            self.avg_distance = float('inf')
            return

        # Find the closest object and the average distance
        self.min_distance = min(valid_ranges)
        self.avg_distance = sum(valid_ranges) / len(valid_ranges)

        # Log results
        self.get_logger().info(f"Closest Object: {self.min_distance:.2f} m | Average Distance: {self.avg_distance:.2f} m")

    def show_lidar_summary(self):
        """Displays the closest detected object and average LiDAR distance."""
        def format_distance(distance):
            return f"{distance:.2f} m" if distance != float('inf') else "No Object Detected"

        self.get_logger().info(f"? Closest Object: {format_distance(self.min_distance)}, "
                               f"? Average Distance: {format_distance(self.avg_distance)}")

    def lidar_callback(self, msg):
        """Processes incoming LiDAR scan data and updates distances correctly."""
        self.get_logger().info("LiDAR callback triggered!")  # Debugging message

        ranges = msg.ranges
        num_readings = len(ranges)

        if num_readings == 0:
            self.get_logger().warn("LiDAR message received, but ranges list is empty!")
            return

        # Extract angle information
        angle_min = msg.angle_min  # Minimum angle (usually negative)
        angle_max = msg.angle_max  # Maximum angle
        angle_increment = msg.angle_increment  # Increment per index

        # Convert degrees to radians for indexing
        front_angle = 0
        back_angle = math.pi  # 180 degrees
        left_angle = math.pi / 2  # 90 degrees
        right_angle = -math.pi / 2  # -90 degrees

        # Convert angles to LiDAR index values
        front_index = min(num_readings - 1, max(0, int((front_angle - angle_min) / angle_increment)))
        back_index = min(num_readings - 1, max(0, int((back_angle - angle_min) / angle_increment)))
        left_index = min(num_readings - 1, max(0, int((left_angle - angle_min) / angle_increment)))
        right_index = min(num_readings - 1, max(0, int((right_angle - angle_min) / angle_increment)))

        # Extract distances, handling 'inf' cases
    def get_distance(index):
        return ranges[index] if not math.isinf(ranges[index]) else float('inf')
        self.front_distance = get_distance(front_index)
        self.back_distance = get_distance(back_index)
        self.left_distance = get_distance(left_index)
        self.right_distance = get_distance(right_index)

        # Log new values for debugging
        self.get_logger().info(f"LiDAR Updated - Front: {self.front_distance:.2f}m, "
                           f"Back: {self.back_distance:.2f}m, "
                           f"Left: {self.left_distance:.2f}m, "
                           f"Right: {self.right_distance:.2f}m")

    def print_lidar_readings(self):

        rclpy.spin_once(self)  # Process LiDAR data
        print("LiDAR Readings:")
        print(f"Front Distance: {node.front_distance} meters")
        print(f"Back Distance: {node.back_distance} meters")
        print(f"Left Distance: {node.left_distance} meters")
        print(f"Right Distance: {node.right_distance} meters")
 
    
 
    def play_buzzer(self, freq, on_time, off_time, repeat):
        msg = BuzzerState(freq=freq, on_time=on_time, off_time=off_time, repeat=repeat)
        self.buzzer_pub.publish(msg)
        self.get_logger().info(f'Playing buzzer at {freq}Hz for {repeat} cycles.')

    def stop_buzzer(self):
        self.play_buzzer(0, 0, 0, 1)
        self.get_logger().info("Stopped the buzzer.")

    def set_color(self, index, red, green, blue):
        msg = RGBStates(states=[RGBState(index=index, red=red, green=green, blue=blue)])
        self.rgb_pub.publish(msg)
        self.get_logger().info(f'Set LED {index} to color R={red}, G={green}, B={blue}')

    def set_all_colors(self, red, green, blue):
        msg = RGBStates(states=[RGBState(index=i, red=red, green=green, blue=blue) for i in range(1, 3)])
        self.rgb_pub.publish(msg)
        self.get_logger().info(f'Set all LEDs to color R={red}, G={green}, B={blue}')
    def blink(self, index, red, green, blue, times=3, delay=0.5):
        """
        Blinks an LED a specified number of times.
        :param index: LED index (1 or 2)
        :param red: Red value (0-255)
        :param green: Green value (0-255)
        :param blue: Blue value (0-255)
        :param times: Number of blinks
        :param delay: Time delay between on/off
        """
        for _ in range(times):
            self.set_color(index, red, green, blue)  # Turn LED on
            time.sleep(delay)
            self.set_color(index, 0, 0, 0)  # Turn LED off
            time.sleep(delay)

        time.sleep(0.1)  # Allow time for the last 'off' signal to process
        self.set_color(index, 0, 0, 0)  # Turn LED off again to ensure it's off
        self.get_logger().info(f'Blinked LED {index} {times} times.')
        
    def print_lidar_distances(self):
        """
        Prints the current LiDAR distances for front, back, left, and right.
        This function should be called after `rclpy.spin_once(node)` in run_commands.py.
        """

        # Format distances, replacing `inf` with "No Object Detected"
        def format_distance(distance):
            return f"{distance:.2f} m" if distance != float('inf') else "No Object Detected"

        self.get_logger().info(f"Front: {format_distance(self.front_distance)}, "
                               f"Back: {format_distance(self.back_distance)}, "
                               f"Left: {format_distance(self.left_distance)}, "
                               f"Right: {format_distance(self.right_distance)}")
    def show_lidar_distances(self):
        """
        Prints LiDAR distances in a formatted way.
        Call this function after `rclpy.spin_once(node)` in run_commands.py.
        """

        self.get_logger().info(f"Front: {self.front_distance:.2f} m, "
                               f"Back: {self.back_distance:.2f} m, "
                               f"Left: {self.left_distance:.2f} m, "
                               f"Right: {self.right_distance:.2f} m")

    def is_obstacle_near(self, direction):
        """
        Check if an obstacle is within the critical distance in a specific direction.
        :param direction: 'front', 'back', 'left', or 'right'
        :return: Boolean indicating if an obstacle is near
        """
        rclpy.spin_once(self)  # Process LiDAR data

        if direction == 'front':
            return self.front_distance < self.critical_distance
        elif direction == 'back':
            return self.back_distance < self.critical_distance
        elif direction == 'left':
            return self.left_distance < self.critical_distance
        elif direction == 'right':
            return self.right_distance < self.critical_distance
        return False
        
    def publish_motor_command(self, motor_states):
        """
        Publishes motor commands to the control topic.
        :param motor_states: List of MotorState messages.
        """
        msg = MotorsState(data=motor_states)
        self.motor_pub.publish(msg)
        self.robot_moving = any(motor.rps != 0 for motor in motor_states)


    def stop_all_motors(self):
        """Stops all motors by setting their speed to 0."""
        motor_states = [MotorState(id=i, rps=0.0) for i in range(1, 5)]
        self.publish_motor_command(motor_states)
        self.get_logger().info('Stopped all motors.')
        self.robot_moving = False  # Ensure movement flag is updated

    def move_with_duration(self, motor_states, duration):
        """
        Moves the robot by publishing motor commands for a specified duration.
        :param motor_states: List of MotorState messages.
        :param duration: Duration (in seconds) for the movement.
        """
        self.publish_motor_command(motor_states)
        self.get_logger().info(f'Movement started for {duration:.2f} seconds...')
        time.sleep(duration)  # Wait for the specified duration
        self.stop_all_motors()

    def rotate_left(self, speed, duration):
        """
        Moves the robot by setting all motors to drive forward.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=speed),  # Front-left forward
            MotorState(id=2, rps=speed),  # Back-left forward
            MotorState(id=3, rps=speed),  # Front-right forward
            MotorState(id=4, rps=speed)   # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)

    def rotate_right(self, speed, duration):
        """
        Moves the robot backward by setting all motors to reverse.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=-speed),  # Front-left backward
            MotorState(id=2, rps=-speed),  # Back-left backward
            MotorState(id=3, rps=-speed),  # Front-right backward
            MotorState(id=4, rps=-speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)

    def move_left(self, speed, duration):
        """
        Moves the robot left by setting appropriate motor directions.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=-speed),  # Front-left backward
            MotorState(id=2, rps=speed),   # Back-left forward
            MotorState(id=3, rps=speed),   # Front-right forward
            MotorState(id=4, rps=-speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)

    def move_right(self, speed, duration):
        """
        Moves the robot right by setting appropriate motor directions.
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        motor_states = [
            MotorState(id=1, rps=speed),   # Front-left forward
            MotorState(id=2, rps=-speed),  # Back-left backward
            MotorState(id=3, rps=-speed),  # Front-right backward
            MotorState(id=4, rps=speed)    # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)

    def move_in_direction(self, degrees, speed, duration):
        """
        Moves the robot in a specified direction using degrees.
        :param degrees: Direction in degrees (0 = forward, 90 = right, etc.).
        :param speed: Speed of the movement.
        :param duration: Duration (in seconds) for the movement.
        """
        radians = math.radians(degrees)

        # Decompose the speed into x and y components
        linear_x = speed * math.cos(radians)
        linear_y = speed * math.sin(radians)
        angular_z = 0.0  # No rotation for directional movement

        # Use mecanum kinematics to calculate motor speeds
        speeds = self.mecanum.set_velocity(linear_x, linear_y, angular_z)

        # Publish motor speeds for the specified duration
        self.publish_motor_command(speeds.data)
        self.get_logger().info(f'Moving in direction {degrees}Â° at {speed} m/s for {duration} seconds...')
        time.sleep(duration)
        self.stop_all_motors()

    def move_forward(self, angular_speed, duration):
        """
        Rotates the robot left (counterclockwise) by applying opposite motor speeds.
        :param angular_speed: Speed of rotation.
        :param duration: Duration (in seconds) for the rotation.
        """
        motor_states = [
            MotorState(id=1, rps=-angular_speed),  # Front-left backward
            MotorState(id=2, rps=-angular_speed),  # Back-left backward
            MotorState(id=3, rps=angular_speed),   # Front-right forward
            MotorState(id=4, rps=angular_speed)    # Back-right forward
        ]
        self.move_with_duration(motor_states, duration)
        self.get_logger().info(f'Rotating left (CCW) at {angular_speed:.2f} RPS for {duration:.2f} seconds.')

    def move_backward(self, angular_speed, duration):
        """
        Rotates the robot right (clockwise) by applying opposite motor speeds.
        :param angular_speed: Speed of rotation.
        :param duration: Duration (in seconds) for the rotation.
        """
        motor_states = [
            MotorState(id=1, rps=angular_speed),   # Front-left forward
            MotorState(id=2, rps=angular_speed),   # Back-left forward
            MotorState(id=3, rps=-angular_speed),  # Front-right backward
            MotorState(id=4, rps=-angular_speed)   # Back-right backward
        ]
        self.move_with_duration(motor_states, duration)
        self.get_logger().info(f'Rotating right (CW) at {angular_speed:.2f} RPS for {duration:.2f} seconds.')


def main(args=None):
    """
    Main function to demonstrate robot control.
    """
    rclpy.init(args=args)
    node = OmniWheelControlNode()

    try:
        # Example commands
        node.move_forward(0.5, 3.0)  # Forward for 3 seconds
        node.move_backward(0.5, 3.0)  # Backward for 3 seconds
        node.move_left(0.5, 2.0)  # Left for 2 seconds
        node.move_right(0.5, 2.0)  # Right for 2 seconds
        node.move_in_direction(45, 0.5, 2.0)  # Diagonal (45Â°) for 2 seconds
        node.rotate_left(0.5, 2.0)  # Rotate CCW for 2 seconds
        node.rotate_right(0.5, 2.0)  # Rotate CW for 2 seconds
    finally:
        node.stop_all_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
