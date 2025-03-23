import os
import cv2
import qrcode
import numpy as np
import queue
import threading
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar

class QRCodeDetector(Node):
    """ ROS2 Node to detect QR codes from an image stream. """

    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(
            Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 1
        )
        self.detected_data = None
        threading.Thread(target=self.process_images, daemon=True).start()

    def image_callback(self, ros_image):
        """ Converts ROS Image message to OpenCV format and adds it to the queue. """
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(cv_image)

    def process_images(self):
        """ Continuously processes images for QR codes. """
        while rclpy.ok():
            try:
                image = self.image_queue.get(timeout=1)
                decoded_objects = pyzbar.decode(image)

                for obj in decoded_objects:
                    self.detected_data = obj.data.decode("utf-8")
                    self.get_logger().info(f"QR Code Detected: {self.detected_data}")
                    return  # Stop after first successful detection

            except queue.Empty:
                continue

    def get_detected_qr_code(self):
        """ Returns the last detected QR code data. """
        return self.detected_data

def generate_qr_code(data, output_file="my_qrcode.jpg"):
    """ Generates a QR code and saves it as an image. """
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=5,
        border=4
    )
    qr.add_data(data)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    img = np.array(img)

    cv2.imshow("Generated QR Code", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv2.imwrite(output_file, img)
    print(f"QR Code saved as {output_file}")

if __name__ == "__main__":
    option = input("Enter '1' to generate QR code, '2' to detect: ")
    if option == "1":
        text = input("Enter text for QR code: ")
        generate_qr_code(text)
    elif option == "2":
        rclpy.init()
        detector = QRCodeDetector()
        rclpy.spin_once(detector)
        print("Detected QR Code:", detector.get_detected_qr_code())
