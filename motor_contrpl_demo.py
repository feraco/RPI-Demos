#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Motor Control Example in ROS2                        **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- This script controls multiple motors using ROS2.
- It sends commands to move motors forward, reverse, or stop.
- On shutdown, it sends a stop command to ensure all motors are halted.

Command Table:
--------------------------------------------------------------------------
| Command Structure                           | Explanation                       |
--------------------------------------------------------------------------
| /ros_robot_controller/set_motor             | Topic to control motors           |
| Message: MotorsState                        | Main message type                 |
|   - data: [MotorState]                      | List of motor states              |
| Sub-Message: MotorState                     | Defines a single motor            |
|   - id: int                                 | Motor ID (1, 2, etc.)             |
|   - rps: float                              | Speed in Revolutions/Sec (RPS)    |
--------------------------------------------------------------------------
| **Custom Commands**                         | **What it Does**                  |
--------------------------------------------------------------------------
| Move Motor Forward                          | Set positive RPS value            |
| Move Motor Reverse                          | Set negative RPS value            |
| Stop Motor                                  | Set RPS value to 0                |
| Send Stop Command on Shutdown               | Ensure all motors stop safely     |
--------------------------------------------------------------------------
"""

import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from ros_robot_controller_msgs.msg import MotorsState, MotorState  # Custom messages for motor control


class MotorControlNode(Node):
    """
    A ROS2 Node that controls multiple motors and sets their speed 
    and direction using the '/set_motor' topic. Stops all motors on exit.
    """
    def __init__(self):
        # Initialize the node with the name 'motor_control_node'
        super().__init__('motor_control_node')

        # Publisher: Sends motor commands to the '/set_motor' topic
        self.motor_pub = self.create_publisher(
            MotorsState, '/ros_robot_controller/set_motor', 10
        )

        # Timer: Calls the timer_callback function every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Motor control states
        self.motor_states = [
            {"id": 1, "rps": 0.2},  # Motor 1: Forward
            {"id": 2, "rps": -0.2},  # Motor 2: Reverse
            {"id": 3, "rps": 0.0}   # Motor 3: Stop
        ]

        # Motor control index to cycle commands
        self.command_index = 0

        # Log that the node has started
        self.get_logger().info('Motor Control Node has started. Sending motor commands.')

    def timer_callback(self):
        """
        Callback function to send motor control commands periodically.
        It cycles through commands to move motors forward, reverse, or stop.
        """
        # Create a MotorsState message
        msg = MotorsState()

        # Create motor states for multiple motors
        for state in self.motor_states:
            motor_state = MotorState()
            motor_state.id = state["id"]  # Set motor ID
            motor_state.rps = state["rps"]  # Set motor speed in RPS
            msg.data.append(motor_state)

        # Publish the motor control message
        self.motor_pub.publish(msg)

        # Log the command being sent
        self.get_logger().info('Sent motor control commands:')
        for motor in self.motor_states:
            self.get_logger().info(
                f'  Motor ID={motor["id"]}, RPS={motor["rps"]}'
            )

        # Update the motor states for the next command cycle
        self.update_motor_states()

    def update_motor_states(self):
        """
        Update motor states to cycle between forward, reverse, and stop.
        """
        # Define a sequence of states: forward, reverse, stop
        sequences = [
            {"id": 1, "rps": 0.2},   # Motor 1 Forward
            {"id": 2, "rps": -0.2},  # Motor 2 Reverse
            {"id": 3, "rps": 0.0}    # Motor 3 Stop
        ]

        # Cycle through commands
        self.command_index = (self.command_index + 1) % len(sequences)
        for i, state in enumerate(self.motor_states):
            state["rps"] = sequences[(self.command_index + i) % len(sequences)]["rps"]

    def stop_all_motors(self):
        """
        Sends a stop command to all motors by setting their RPS to 0.
        """
        msg = MotorsState()
        for motor_id in range(1, 4):  # Assuming motor IDs 1 to 3
            motor_state = MotorState()
            motor_state.id = motor_id
            motor_state.rps = 0.0  # Stop the motor
            msg.data.append(motor_state)

        self.motor_pub.publish(msg)
        self.get_logger().info('Stop command sent to all motors.')


def main(args=None):
    """
    Main function that initializes the ROS2 node and starts spinning.
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Create an instance of the MotorControlNode
    node = MotorControlNode()

    try:
        # Keep the node alive to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully handle shutdown when Ctrl+C is pressed
        node.get_logger().info('Keyboard interrupt detected. Stopping all motors...')
        node.stop_all_motors()
    finally:
        # Ensure all motors are stopped and shut down the node
        node.destroy_node()
        rclpy.shutdown()
        print("Motor Control Node has been shut down safely.")


if __name__ == '__main__':
    main()
