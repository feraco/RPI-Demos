#!/usr/bin/env python3
# encoding: utf-8

"""
**********************************************************
** IMU Subscriber Example in ROS2                       **
** Developed by: Frederick Feraco                       **
** InnovatED STEM / DroneBlocks Land, Air and Sea       **
**********************************************************

Description:
- This script simplifies subscribing to IMU data in ROS2.
- It uses helper methods to extract and display IMU data:
  1. Orientation (Quaternion)
  2. Angular Velocity
  3. Linear Acceleration

Simplified Commands:
--------------------------------------------------------------
| Command                        | What It Does               |
--------------------------------------------------------------
| get_orientation(msg)           | Extracts IMU orientation   |
| get_angular_velocity(msg)      | Extracts angular velocity  |
| get_linear_acceleration(msg)   | Extracts linear accel.     |
| display_imu_data(msg)          | Displays all IMU data      |
--------------------------------------------------------------

Default Topic:
- **/ros_robot_controller/imu_raw** (Can be customized via parameters)

**********************************************************
"""

import rclpy  # ROS2 library
from rclpy.node import Node  # Base class for ROS2 nodes
from sensor_msgs.msg import Imu  # IMU message type for sensor data


class IMUSubscriber(Node):
    """
    A ROS2 Node that subscribes to IMU data and provides helper methods 
    to simplify orientation, angular velocity, and linear acceleration extraction.
    """
    def __init__(self):
        super().__init__('imu_subscriber')  # Initialize the node with a name

        # Declare a parameter for the IMU topic with a default value
        self.declare_parameter('imu_topic', '/ros_robot_controller/imu_raw')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # Create a subscription to the IMU topic
        self.subscription = self.create_subscription(
            Imu,  # Message type: sensor_msgs/Imu
            self.imu_topic,  # IMU topic to subscribe to
            self.listener_callback,  # Callback function to process messages
            10  # Queue size for message buffering
        )
        self.get_logger().info(f'Subscribed to IMU data on topic: {self.imu_topic}')

    def listener_callback(self, msg):
        """
        Callback function that processes incoming IMU messages.
        Simplifies extraction using helper methods.
        """
        self.display_imu_data(msg)  # Display all IMU data

    def get_orientation(self, msg):
        """
        Extracts the IMU orientation (quaternion) from the message.
        """
        return msg.orientation

    def get_angular_velocity(self, msg):
        """
        Extracts the IMU angular velocity (x, y, z) from the message.
        """
        return msg.angular_velocity

    def get_linear_acceleration(self, msg):
        """
        Extracts the IMU linear acceleration (x, y, z) from the message.
        """
        return msg.linear_acceleration

    def display_imu_data(self, msg):
        """
        Displays the IMU data: orientation, angular velocity, and linear acceleration.
        """
        orientation = self.get_orientation(msg)
        angular_velocity = self.get_angular_velocity(msg)
        linear_acceleration = self.get_linear_acceleration(msg)

        # Log IMU data in a clean format
        self.get_logger().info('--- IMU Data ---')
        self.get_logger().info(f'Orientation: x={orientation.x:.2f}, y={orientation.y:.2f}, z={orientation.z:.2f}, w={orientation.w:.2f}')
        self.get_logger().info(f'Angular Velocity: x={angular_velocity.x:.2f}, y={angular_velocity.y:.2f}, z={angular_velocity.z:.2f}')
        self.get_logger().info(f'Linear Acceleration: x={linear_acceleration.x:.2f}, y={linear_acceleration.y:.2f}, z={linear_acceleration.z:.2f}')


def main(args=None):
    """
    Main function to initialize the IMU subscriber node.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python library
    node = IMUSubscriber()  # Create an instance of the IMU Subscriber node

    try:
        # Keep the node alive to process incoming messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle clean shutdown on Ctrl+C
        node.get_logger().info('Shutting down IMU Subscriber Node...')
    finally:
        # Destroy the node and shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()
        print("IMU Subscriber Node has been safely shut down.")


if __name__ == '__main__':
    main()
