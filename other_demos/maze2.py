import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from time import sleep

class MazeDrawer(Node):
    def __init__(self):
        super().__init__('maze_drawer')

        # Service Clients
        self.cli_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')

        # Wait for services to be available
        self.cli_teleport.wait_for_service()
        self.cli_pen.wait_for_service()

        self.draw_maze()

    def teleport(self, x, y, theta):
        """Teleport the turtle to a specific position."""
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.cli_teleport.call(req)
        sleep(0.1)  # Allow time for the teleport to process

    def set_pen(self, r, g, b, width, off):
        """Set the pen color, width, and state (on/off)."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.cli_pen.call(req)
        sleep(0.1)  # Allow time for the pen to adjust

    def draw_line(self, x_start, y_start, x_end, y_end):
        """Draw a line from start to end."""
        self.set_pen(0, 0, 255, 3, 0)  # Set blue pen with width 3
        self.teleport(x_start, y_start, 0.0)
        self.set_pen(0, 0, 255, 3, 1)  # Turn off pen to teleport cleanly
        self.teleport(x_end, y_end, 0.0)
        self.set_pen(0, 0, 255, 3, 0)  # Turn pen back on

    def draw_maze(self):
        """Draw the maze structure."""
        # Clear the canvas
        self.set_pen(0, 0, 0, 1, 1)  # Pen off
        self.teleport(5.5, 5.5, 0.0)

        # Draw maze boundary
        self.draw_line(1.0, 9.0, 9.0, 9.0)  # Top wall
        self.draw_line(1.0, 1.0, 9.0, 1.0)  # Bottom wall
        self.draw_line(1.0, 1.0, 1.0, 9.0)  # Left wall
        self.draw_line(9.0, 1.0, 9.0, 9.0)  # Right wall

        # Add internal maze walls
        self.draw_line(5.0, 1.0, 5.0, 5.0)  # Vertical wall
        self.draw_line(3.0, 5.0, 7.0, 5.0)  # Horizontal wall
        self.get_logger().info("Maze has been drawn!")

def main(args=None):
    rclpy.init(args=args)
    node = MazeDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
