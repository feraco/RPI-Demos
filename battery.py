#!/usr/bin/env python3
# encoding: utf-8

from robot_controller_utils import BatteryMonitor
import rclpy

def main():
    # Initialize the ROS2 Python library
    rclpy.init()

    try:
        # Create an instance of the BatteryMonitor
        battery_monitor = BatteryMonitor()
        battery_monitor.get_logger().info("Listening for battery voltage data...")

        # Listen for 3 seconds
        rclpy.spin_once(battery_monitor, timeout_sec=3.0)
        voltage = battery_monitor.get_latest_voltage()

        if voltage:
            battery_monitor.get_logger().info(f"Battery Voltage: {voltage} mV")
        else:
            battery_monitor.get_logger().info("No battery voltage data received yet.")

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup: destroy nodes and shutdown ROS2
        battery_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
