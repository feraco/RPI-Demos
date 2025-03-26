import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from threading import Thread, Event

# Global to store the result
_odom_result = None

def get_odom_variable(field_path='pose.pose.position.x', topic='/odom'):
    """
    Fetch one odometry value from a specified field path like 'pose.pose.position.x'
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
            rclpy.shutdown()

    def spin_node():
        rclpy.init()
        node = OneShotOdom()
        rclpy.spin(node)

    # Run ROS spin in background
    thread = Thread(target=spin_node)
    thread.start()

    # Wait for message
    while rclpy.ok() and _odom_result is None:
        pass

    return _odom_result
