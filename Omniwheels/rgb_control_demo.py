#!/usr/bin/env python3
import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from ros_robot_controller_msgs.msg import RGBState, RGBStates  # Message types for controlling RGB LEDs

"""
**********************************************************
************ RGB LED Commands and Their Explanation *******
**********************************************************

1. Set Single LED to Specific Color:
   - Command: Set the RGB values of a single LED.
   - Example: [RGBState(index=1, red=255, green=0, blue=0)]  # LED 1 set to Red.

2. Set Multiple LEDs to the Same Color:
   - Command: Set the same RGB values for all LEDs.
   - Example: 
       [
           RGBState(index=1, red=0, green=255, blue=0),  # LED 1 set to Green
           RGBState(index=2, red=0, green=255, blue=0)   # LED 2 set to Green
       ]

3. Cycle Through Colors:
   - Command: Use a predefined list of RGB colors (Red, Green, Blue, etc.) to change LED colors periodically.
   - Example: Switch from Red → Green → Blue → Yellow → Cyan → Magenta → Off.

4. Turn Off All LEDs:
   - Command: Set all RGB values to 0 to turn off the LEDs.
   - Example: 
       [
           RGBState(index=1, red=0, green=0, blue=0),  # LED 1 off
           RGBState(index=2, red=0, green=0, blue=0)   # LED 2 off
       ]

5. Set LEDs to Random Colors:
   - Command: Use random RGB values for each LED to create a colorful effect.
   - Example:
       [
           RGBState(index=1, red=123, green=45, blue=200),  # Random color
           RGBState(index=2, red=220, green=100, blue=50)   # Random color
       ]

6. Set LEDs with Gradient (Advanced):
   - Command: Smoothly transition LED colors by gradually increasing/decreasing RGB values.
   - Example: Gradually move from Red → Orange → Yellow → Green.

**********************************************************
"""

class RGBControllerNode(Node):
    """
    A ROS2 Node that publishes RGB color states to control LEDs on a hardware board.
    """
    def __init__(self):
        # Initialize the node with the name 'rgb_controller'
        super().__init__('rgb_controller')

        # Create a publisher to send RGBStates messages to the '/ros_robot_controller/set_rgb' topic
        self.publisher_ = self.create_publisher(RGBStates, '/ros_robot_controller/set_rgb', 10)

        # Create a timer that calls the timer_callback function every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Log a message indicating the node has started
        self.get_logger().info('RGB Controller Node has been started.')

        # Initialize a color index to cycle through the colors
        self.current_color_index = 0

    def timer_callback(self):
        """
        This callback function is called at a fixed interval (1 second).
        It cycles through predefined colors and publishes them to the RGB LEDs.
        """
        # Define a list of colors as RGB tuples
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (0, 255, 255),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 0, 0)       # Off (Black)
        ]

        # Select the next color based on the current index
        color = colors[self.current_color_index % len(colors)]
        self.current_color_index += 1  # Move to the next color in the list

        # Create the RGBStates message
        msg = RGBStates()
        msg.states = [
            RGBState(index=1, red=color[0], green=color[1], blue=color[2]),  # LED 1
            RGBState(index=2, red=color[0], green=color[1], blue=color[2])   # LED 2
        ]

        # Publish the message to set the LEDs' colors
        self.publisher_.publish(msg)

        # Log the current color being sent
        self.get_logger().info(f'Setting LEDs to color R={color[0]}, G={color[1]}, B={color[2]}')

def main(args=None):
    """
    The main function that initializes the ROS2 node and starts spinning.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the RGBControllerNode
    node = RGBControllerNode()

    try:
        # Keep the node alive and spinning to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle the node shutdown gracefully when Ctrl+C is pressed
        node.get_logger().info('Shutting down RGB Controller Node')
    finally:
        # Destroy the node and shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
