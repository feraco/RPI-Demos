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

        self.get_logger().info(f'Moving turtle: linear={linear}, angular={angular}, duration={duration}s')

        # Publish commands for the specified duration
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)  # Small delay to prevent spamming

        # Stop the turtle after the movement
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Stopped turtle.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        # Send a series of movement commands
        node.move_turtle(linear=2.0, angular=0.0, duration=2.0)  # Move forward
        node.move_turtle(linear=0.0, angular=1.5, duration=2.0)  # Rotate in place
        node.move_turtle(linear=1.0, angular=1.0, duration=3.0)  # Move in a curve
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
