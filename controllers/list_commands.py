import rclpy
from omni_robot_controller import OmniWheelControlNode  # Ensure this matches y>

def run_commands(node): 
        node.move_in_direction(0, 2.0, 1)    
        node.move_in_direction(180, 2.0, 1)
        node.move_in_direction(0, 3.0, 1)    
        node.move_in_direction(180, 3.0, 1)
        #node.set_color(1, 0, 0, 0)