import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import queue
import time

class ColorTracker(Node):
    def __init__(self, color_name='red'):
        super().__init__('color_tracker_node')
        self.bridge = CvBridge()
        self.color_name = color_name
        self.image_queue = queue.Queue(maxsize=1)
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            10
        )
        self.processed_frame = None
        self.detected_objects = []

        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),
            'green': ([35, 100, 100], [85, 255, 255], None, None),
            'blue': ([100, 100, 100], [140, 255, 255], None, None),
            'yellow': ([20, 100, 100], [35, 255, 255], None, None)
        }

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(frame)

    def process_frame(self, frame):
        self.detected_objects = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1, upper1, lower2, upper2 = self.color_ranges[self.color_name]

        mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
        if lower2 is not None and upper2 is not None:
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
            self.detected_objects.append({'x': x, 'y': y, 'radius': radius, 'area': area})

        self.processed_frame = processed_frame

    def is_color_detected(self, min_area=500):
        """Returns True if any object of the current color is detected above a certain area."""
        return any(obj['area'] >= min_area for obj in self.detected_objects)

    def get_detected_objects_info(self):
        """Returns a formatted string with information about each detected object."""
        if not self.detected_objects:
            return "No objects detected."
        info = []
        for i, obj in enumerate(self.detected_objects):
            info.append(
                f"Object {i+1}: x={obj['x']:.1f}, y={obj['y']:.1f}, radius={obj['radius']:.1f}, area={obj['area']:.1f}"
            )
        return "\n".join(info)



import matplotlib.pyplot as plt
from IPython.display import display, clear_output

def run_color_tracking(color='red', duration=5):
    if not rclpy.ok():
        rclpy.init()

    node = ColorTracker(color_name=color)
    start = time.time()

    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.1)

        if not node.image_queue.empty():
            frame = node.image_queue.get()
            node.process_frame(frame)

            if node.is_color_detected():
                print(f"✅ {color.upper()} detected!")
                print(node.get_detected_objects_info())

            # Display inline
            rgb = cv2.cvtColor(node.processed_frame, cv2.COLOR_BGR2RGB)
            plt.figure(figsize=(8, 6))
            plt.imshow(rgb)
            plt.title(f"Tracking: {color}")
            plt.axis('off')
            clear_output(wait=True)
            display(plt.gcf())
            plt.close()

        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

# Notebook-friendly wrapper
def is_color_detected(color='red', duration=5, min_area=500):
    """
    Runs the color tracker for a specific color and checks if the color is detected.
    Parameters:
        color (str): Color name to track (e.g., 'red', 'blue')
        duration (int): How many seconds to run the detection loop
        min_area (int): Minimum contour area to be considered a valid detection
    Returns:
        bool: True if color was detected, False otherwise
    """
    if not rclpy.ok():
        rclpy.init()

    node = ColorTracker(color_name=color)
    start = time.time()
    detected = False

    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not node.image_queue.empty():
            frame = node.image_queue.get()
            node.process_frame(frame)
            if node.is_color_detected(min_area):
                print(f"{color.upper()} detected!")
                print(node.get_detected_objects_info())
                detected = True

    node.destroy_node()
    rclpy.shutdown()
    return detected
