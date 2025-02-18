import rclpy
from turtle_controller import TurtleController  # Import TurtleController class

rclpy.init()
turtle = TurtleController()

try:
    turtle.move_turtle(2.0, 0.0, 2.0)  # Move forward
    turtle.move_turtle(0.0, 1.5, 1.5)  # Turn right
    turtle.move_turtle(1.0, 0.5, 3.0)  # Move in a curve
except KeyboardInterrupt:
    turtle.get_logger().info('Shutting down.')

turtle.destroy_node()
rclpy.shutdown()
