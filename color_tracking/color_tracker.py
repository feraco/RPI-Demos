
#!/usr/bin/env python3
# encoding: utf-8

import cv2
import numpy as np
import queue
import rclpy
import threading
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorTracker(Node):
    def __init__(self, color_name='red'):
        rclpy.init()
        super().__init__('color_tracker_node')

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True

        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),
            'green': ([35, 100, 100], [85, 255, 255], None, None),
            'blue': ([100, 100, 100], [140, 255, 255], None, None),
            'yellow': ([20, 100, 100], [35, 255, 255], None, None)
        }

        self.current_color = color_name
        self.detected_objects = []
        self.processed_frame = None

        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            1
        )

        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(cv_image)

    def process_frame(self, frame):
        self.detected_objects = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1, upper1, lower2, upper2 = self.color_ranges[self.current_color]

        mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 and upper2:
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        processed_frame = frame.copy()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(processed_frame, center, radius, (0, 255, 0), 2)
            cv2.circle(processed_frame, center, 5, (0, 0, 255), -1)
            self.detected_objects.append({
                'x': x,
                'y': y,
                'radius': radius,
                'area': area
            })

        cv2.putText(processed_frame, f"Tracking: {self.current_color} ({len(self.detected_objects)} objects)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        self.processed_frame = processed_frame

    def main(self):
        while self.running:
            try:
                frame = self.image_queue.get(timeout=1)
                self.process_frame(frame)
                cv2.imshow('Color Tracker', self.processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except queue.Empty:
                continue

        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    tracker = ColorTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        print("Shutting down...")
        tracker.running = False
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
