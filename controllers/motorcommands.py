#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Motor Control Example in ROS2                        **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- This script simplifies motor control commands in ROS2.
- You can move all motors forward, backward, left, or right 
  using easy-to-use functions with speed as a parameter.

Command Table:
--------------------------------------------------------------------------
| Command                                | Explanation                           |
--------------------------------------------------------------------------
| move_all_motors(speed)                 | Move all motors at a specified speed |
| stop_all_motors()                      | Stop all motors                      |
| move_forward(speed)                    | Move all motors forward              |
| move_backward(speed)                   | Move all motors backward             |
| move_left(speed)                       | Move motors for leftward motion      |
| move_right(speed)                      | Move motors for rightward motion     |
--------------------------------------------------------------------------
"""

import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from ros_robot_controller_msgs.msg import MotorsState, MotorState  # Custom messages for motor control


class MotorControlNode(Node):
    """
    A ROS2 Node that provides easy-to-use methods to control motors.
    Supports forward, backward, left, right, and stop commands.
    """
    def __init__(self):
        # Initialize the node with the name 'motor_control_node'
        super().__init__('motor_control_node')

        # Publisher: Sends motor control messages
        self.motor_pub = self.create_publisher(
            MotorsState, '/ros_robot_controller/set_motor', 10
        )

        # Log that the node has started
        self.get_logger().info('Motor Control Node has started. Ready for commands.')

    def publish_motor_command(self, motor_states):
        """
        Publishes a list of MotorState commands to the motor control topic.
        """
        msg = MotorsState()
        msg.data = motor_states  # Add the list of motor states
        self.motor_pub.publish(msg)

    def move_all_motors(self, speed):
        """
        Move all motors at the specified speed.
        Positive speed = forward, Negative speed = reverse.
        """
        motor_states = [
            MotorState(id=1, rps=speed),
            MotorState(id=2, rps=speed),
            MotorState(id=3, rps=speed),
            MotorState(id=4, rps=speed)
        ]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f'Moving all motors at {speed:.2f} RPS.')

    def stop_all_motors(self):
        """
        Stops all motors by setting their speed (RPS) to 0.
        """
        self.move_all_motors(0.0)
        self.get_logger().info('Stopped all motors.')

    def move_forward(self, speed):
        """
        Move all motors forward at the specified speed.
        """
        self.move_all_motors(speed)

    def move_backward(self, speed):
        """
        Move all motors backward at the specified speed.
        """
        self.move_all_motors(-speed)

    def move_left(self, speed):
        """
        Moves the robot left by reversing one side and moving the other forward.
        """
        motor_states = [
            MotorState(id=1, rps=-speed),  # Left motors reverse
            MotorState(id=2, rps=speed),   # Right motors forward
            MotorState(id=3, rps=-speed),  # Left motors reverse
            MotorState(id=4, rps=speed)    # Right motors forward
        ]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f'Moving left at {speed:.2f} RPS.')

    def move_right(self, speed):
        """
        Moves the robot right by reversing the opposite sides.
        """
        motor_states = [
            MotorState(id=1, rps=speed),   # Left motors forward
            MotorState(id=2, rps=-speed),  # Right motors reverse
            MotorState(id=3, rps=speed),   # Left motors forward
            MotorState(id=4, rps=-speed)   # Right motors reverse
        ]
        self.publish_motor_command(motor_states)
        self.get_logger().info(f'Moving right at {speed:.2f} RPS.')


def main(args=None):
    """
    Main function to initialize and control the MotorControlNode.
    """
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        # Example usage of the easy-to-use commands
        node.get_logger().info('Example: Moving Forward for 3 seconds...')
        node.move_forward(0.5)  # Move forward at 0.5 RPS
        rclpy.spin_once(node, timeout_sec=3.0)  # Run for 3 seconds

        node.get_logger().info('Example: Moving Backward for 3 seconds...')
        node.move_backward(0.5)  # Move backward at 0.5 RPS
        rclpy.spin_once(node, timeout_sec=3.0)

        node.get_logger().info('Example: Moving Left for 3 seconds...')
        node.move_left(0.5)  # Move left at 0.5 RPS
        rclpy.spin_once(node, timeout_sec=3.0)

        node.get_logger().info('Example: Moving Right for 3 seconds...')
        node.move_right(0.5)  # Move right at 0.5 RPS
        rclpy.spin_once(node, timeout_sec=3.0)

        node.get_logger().info('Example: Stopping all motors...')
        node.stop_all_motors()  # Stop all motors
        rclpy.spin_once(node, timeout_sec=1.0)

    except KeyboardInterrupt:
        # Stop motors on shutdown
        node.get_logger().info('Shutting down. Stopping all motors.')
        node.stop_all_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Motor Control Node has been shut down safely.")


if __name__ == '__main__':
    main()
