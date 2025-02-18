#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** Servo Control Example in ROS2                        **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- This script controls a servo motor using ROS2 and the 
  `/ros_robot_controller/bus_servo/set_position` topic.
- The servo oscillates between two positions (0 to 1000) automatically.

Command Table:
--------------------------------------------------------------------------
| Command Structure                           | Explanation                |
--------------------------------------------------------------------------
| /ros_robot_controller/bus_servo/set_position | Topic to control servos   |
| Message: ServosPosition                     | Main message type         |
|   - duration: float                         | Time for the movement     |
|   - position: [ServoPosition]               | List of servo positions   |
| Sub-Message: ServoPosition                  | Defines a single servo    |
|   - id: int                                 | Servo ID (e.g., 1, 2, 3)  |
|   - position: int                           | Target position (0–1000)  |
--------------------------------------------------------------------------

Parts of the Message:
1. **duration**: Controls how fast or slow the servo moves. Smaller values = faster movement.
2. **id**: Specifies which servo to control. Each servo has a unique ID (e.g., 1).
3. **position**: Defines the target position (0 = minimum, 1000 = maximum range).

Hardware Requirements:
- A bus servo connected to the robot's controller board.
- ROS2 setup and the appropriate custom message definitions.

**********************************************************
"""

import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition  # Custom messages for servo control


class BusServoControlNode(Node):
    """
    A ROS2 Node that controls a bus servo by publishing messages to 
    move it back and forth between two positions.
    """
    def __init__(self):
        # Initialize the node with the name 'bus_servo_control_node'
        super().__init__('bus_servo_control_node')

        # Publisher: Sends servo position commands to the '/bus_servo/set_position' topic
        self.publisher = self.create_publisher(
            ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10
        )

        # Timer: Calls the timer_callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize position and movement direction
        self.position = 500  # Starting position (middle)
        self.moving_up = True  # Direction flag

        # Log that the node has started
        self.get_logger().info('Bus Servo Control Node has started. Publishing commands.')

    def timer_callback(self):
        """
        Callback function to send periodic commands to the bus servo.
        It oscillates the servo between positions 0 and 1000.
        """
        # Create a ServosPosition message
        msg = ServosPosition()
        msg.duration = 0.5  # Duration for the movement (0.5 seconds)

        # Create a ServoPosition sub-message for a single servo
        servo_position = ServoPosition()
        servo_position.id = 1  # ID of the servo being controlled
        servo_position.position = self.position  # Target position for the servo

        # Add the servo position to the ServosPosition message
        msg.position = [servo_position]

        # Publish the message
        self.publisher.publish(msg)

        # Log the command being sent
        self.get_logger().info(
            f'Sent command: servo_id={servo_position.id}, position={self.position}, duration={msg.duration}s'
        )

        # Update the position to oscillate the servo
        if self.moving_up:
            self.position += 200  # Move upwards
            if self.position >= 1000:  # If upper limit reached, reverse direction
                self.moving_up = False
        else:
            self.position -= 200  # Move downwards
            if self.position <= 0:  # If lower limit reached, reverse direction
                self.moving_up = True


def main(args=None):
    """
    Main function that initializes the node and starts spinning.
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Create an instance of the BusServoControlNode
    node = BusServoControlNode()

    try:
        # Keep the node alive and spinning to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully shutdown when Ctrl+C is pressed
        node.get_logger().info('Bus Servo Control Node is shutting down.')
    finally:
        # Destroy the node and shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
