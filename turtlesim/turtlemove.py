import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move_turtle(self, linear, angular, duration):
        """Send movement commands to the turtle."""
        msg = Twist()
        msg.linear.x = linear   # Forward/backward speed
        msg.angular.z = angular # Rotational speed

        self.get_logger().info(f'Moving: linear={linear}, angular={angular}, duration={duration}s')

        # Publish commands for the specified duration
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)  # Prevent spamming

        # Stop the turtle after movement
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Stopped.')
