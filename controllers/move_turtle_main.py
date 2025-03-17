import rclpy
from move_turtle_logic import TurtleController
from turtle_commands import run_commands  # Import the run_commands function

def main(args=None):
    rclpy.init(args=args)

    # Create a TurtleController instance
    turtle_controller = TurtleController()

    try:
        # Execute commands from the external file
        run_commands(turtle_controller)
    except KeyboardInterrupt:
        pass

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
