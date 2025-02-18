#!/usr/bin/env python3
import rclpy  # ROS2 Python library
from rclpy.node import Node  # Base class for ROS2 nodes
from std_msgs.msg import UInt16  # Message type for battery voltage (UInt16 = unsigned 16-bit integer)

"""
**********************************************************
********** Useful Battery Commands and Explanations *******
**********************************************************

1. **Monitor Battery Voltage (Default Command):**
   - This script reads battery voltage from the `/ros_robot_controller/battery` topic.
   - Message Type: UInt16
   - Example Output: "Received battery voltage: 7200 mV"

2. **Set a Low Voltage Alert Threshold:**
   - Compare the battery voltage to a defined threshold.
   - Example:
       Alert when voltage < 7000 mV.

3. **Calculate Battery Percentage (Assuming 2S LiPo Battery):**
   - Approximate battery percentage from the voltage.
   - Formula: 
       Percentage = ((Voltage - 7.0) / 1.0) * 100
       Where:
           - 7.0 V = Discharged (2S battery)
           - 8.4 V = Fully charged (2S battery)

4. **Log Voltage Changes Over Time:**
   - Track battery voltage changes and print the difference to analyze the discharge rate.

5. **Detect Critical Voltage Drop:**
   - Identify if the battery voltage suddenly drops (e.g., voltage drops > 100 mV in one reading).

**********************************************************
"""

class BatteryVoltageSubscriberNode(Node):
    """
    A ROS2 Node that subscribes to the '/ros_robot_controller/battery' topic 
    and logs the battery voltage with additional monitoring features.
    """
    def __init__(self):
        # Initialize the node with the name 'battery_voltage_subscriber_node'
        super().__init__('battery_voltage_subscriber_node')

        # Subscribe to the battery voltage topic
        self.subscription = self.create_subscription(
            UInt16,  # Message type for battery voltage
            '/ros_robot_controller/battery',  # Battery topic name
            self.battery_voltage_callback,  # Callback function when data is received
            10  # Queue size
        )
        self.subscription

        # Initialize variables for tracking voltage
        self.last_voltage = None  # Stores the last battery voltage reading
        self.low_voltage_threshold = 7000  # Example threshold: 7000 mV (7.0 V for a 2S battery)

        self.get_logger().info('Battery Voltage Subscriber Node has started.')

    def battery_voltage_callback(self, msg):
        """
        Callback function for handling incoming battery voltage messages.
        """
        current_voltage = msg.data  # Extract voltage value from the message
        self.get_logger().info(f'Received battery voltage: {current_voltage} mV')

        # Check if the voltage is below a defined threshold
        if current_voltage < self.low_voltage_threshold:
            self.get_logger().warn(f'Low Battery Alert! Voltage = {current_voltage} mV')

        # Calculate and display the battery percentage (2S LiPo approximation)
        battery_percentage = self.calculate_battery_percentage(current_voltage)
        self.get_logger().info(f'Estimated Battery Percentage: {battery_percentage:.2f}%')

        # Track voltage change and detect sudden drops
        if self.last_voltage is not None:
            voltage_drop = self.last_voltage - current_voltage
            if abs(voltage_drop) > 100:  # Example: Detect drops greater than 100 mV
                self.get_logger().warn(f'Warning: Sudden Voltage Drop Detected! Drop = {voltage_drop} mV')
        self.last_voltage = current_voltage

    def calculate_battery_percentage(self, voltage):
        """
        Approximates battery percentage for a 2S LiPo battery (7.0 V - 8.4 V).
        """
        min_voltage = 7000  # 7.0 V = Discharged
        max_voltage = 8400  # 8.4 V = Fully charged
        percentage = max(0, min(100, ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100))
        return percentage

def main(args=None):
    """
    The main function that initializes the ROS2 node and starts spinning.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the BatteryVoltageSubscriberNode
    node = BatteryVoltageSubscriberNode()

    try:
        # Keep the node alive and spinning to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle the node shutdown gracefully when Ctrl+C is pressed
        node.get_logger().info('Shutting down Battery Voltage Subscriber Node')
    finally:
        # Destroy the node and shutdown ROS2
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
