#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',  # Adjust topic if needed
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_received = None
        self.save_path = os.path.expanduser("~/Pictures/captured_image.jpg")
        self.get_logger().info(f'Image capture node initialized. Will save to: {self.save_path}')

    def image_callback(self, msg):
        """ Callback function to process image messages """
        self.image_received = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Save the image
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        cv2.imwrite(self.save_path, self.image_received)
        
        self.get_logger().info(f'Image saved to {self.save_path}')

    def capture_photo(self):
        """ Trigger a single image capture """
        if self.image_received is not None:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            cv2.imwrite(self.save_path, self.image_received)
            self.get_logger().info(f'Captured and saved image to {self.save_path}')
        else:
            self.get_logger().warn("No image received yet.")

def main():
    rclpy.init()
    node = ImageCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
