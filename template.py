#!/usr/bin/env python3

import rclpy
from omni_robot_controller import OmniWheelControlNode

def main():
    rclpy.init()
    node = OmniWheelControlNode()

    try:
        # Example commands
       
        node.rotate_right(5.0, 2.0)
        node.rotate_left(5.0, 2.0)
#        node.move_in_direction(90, 1.0, .3)  
 #       node.rotate_left(4.0, 0.5)
#        node.rotate_right(4.0, 0.5)
      #  node.move_backward(2.0, 2)

        

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down. Stopping all motors.')
        node.stop_all_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
