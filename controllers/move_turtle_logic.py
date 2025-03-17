from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def send_command(self, linear, angular, duration):
        """Send a movement command to the turtle."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            time.sleep(0.1)

        # Stop the turtle after the movement
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
