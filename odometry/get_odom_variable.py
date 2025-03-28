
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from threading import Event

# Global to store the result
_odom_result = None

def get_odom_variable(field_path='pose.pose.position.x', topic='/odom', rclpy_context=None):
    """
    Fetch one odometry value from a specified field path like 'pose.pose.position.x'.
    Reuses existing rclpy context; assumes rclpy.init() was already called.
    """
    global _odom_result

    class OneShotOdom(Node):
        def __init__(self):
            super().__init__('one_shot_odom')
            self.subscription = self.create_subscription(
                Odometry,
                topic,
                self.callback,
                10
            )
            self.received = Event()

        def callback(self, msg):
            # Dynamic field access (e.g. msg.pose.pose.position.x)
            value = msg
            for attr in field_path.split('.'):
                value = getattr(value, attr)
            global _odom_result
            _odom_result = value
            self.received.set()

    # Create node and spin until one message is received
    node = OneShotOdom()
    while not node.received.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    return _odom_result
