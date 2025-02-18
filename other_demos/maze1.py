import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen

class MazeDrawer(Node):
    def __init__(self):
        super().__init__('maze_drawer')

        # Wait for services to become available
        self.cli_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.cli_teleport.wait_for_service()
        self.cli_pen.wait_for_service()

        self.draw_maze()

    def teleport(self, x, y, theta):
        """Teleport the turtle to a specific position."""
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.cli_teleport.call_async(req)

    def set_pen(self, r, g, b, width, off):
        """Set pen color, width, and state (on/off)."""
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.cli_pen.call_async(req)

    def draw_line(self, x_start, y_start, x_end, y_end):
        """Draw a line from start to end coordinates."""
        self.set_pen(0, 0, 255, 2, 0)  # Set pen color (blue) and on
        self.teleport(x_start, y_start, 0.0)
        self.teleport(x_end, y_end, 0.0)

    def draw_maze(self):
        """Draw the maze on the canvas."""
        # Clear the screen and set up initial pen
        self.set_pen(0, 0, 0, 0, 1)  # Pen off
        self.teleport(5.5, 5.5, 0.0)  # Center turtle

        # Maze walls (example)
        self.draw_line(1.0, 9.0, 9.0, 9.0)  # Top wall
        self.draw_line(1.0, 1.0, 9.0, 1.0)  # Bottom wall
        self.draw_line(1.0, 1.0, 1.0, 9.0)  # Left wall
        self.draw_line(9.0, 1.0, 9.0, 9.0)  # Right wall
        self.draw_line(5.0, 1.0, 5.0, 5.0)  # Vertical divider

        self.get_logger().info("Maze drawn!")

def main(args=None):
    rclpy.init(args=args)
    maze_drawer = MazeDrawer()
    rclpy.spin(maze_drawer)
    maze_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

