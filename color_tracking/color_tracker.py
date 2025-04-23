#!/usr/bin/env python3
# encoding: utf-8
"""
Color Tracking Module

This module provides functionality for tracking colors using a camera.
It includes classes and functions for detecting and tracking colored objects.
"""

import cv2
import numpy as np
import time
import threading

class ColorTracker:
    """
    A class for tracking colored objects in a video stream.
    
    This class provides methods to detect and track objects of a specific color
    using HSV color space filtering.
    """
    
    def __init__(self, camera_index=0):
        """
        Initialize the ColorTracker.
        
        Args:
            camera_index (int): Index of the camera to use (default: 0)
        """
        self.camera_index = camera_index
        self.cap = None
        self.running = False
        self.frame = None
        self.processed_frame = None
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),  # Red spans two ranges in HSV
            'green': ([35, 100, 100], [85, 255, 255], None, None),
            'blue': ([100, 100, 100], [140, 255, 255], None, None),
            'yellow': ([20, 100, 100], [35, 255, 255], None, None)
        }
        self.current_color = 'red'
        self.detected_objects = []
        self.thread = None
    
    def start(self):
        """Start the color tracking process."""
        if self.running:
            return
            
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")
            
        self.running = True
        self.thread = threading.Thread(target=self._process_frames)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop the color tracking process."""
        self.running = False
        if self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()
    
    def set_color(self, color_name):
        """
        Set the color to track.
        
        Args:
            color_name (str): Name of the color to track ('red', 'green', 'blue', 'yellow')
        """
        if color_name in self.color_ranges:
            self.current_color = color_name
            return True
        return False
    
    def add_custom_color(self, name, hsv_lower, hsv_upper, hsv_lower2=None, hsv_upper2=None):
        """
        Add a custom color range to track.
        
        Args:
            name (str): Name of the custom color
            hsv_lower (list): Lower bound of HSV values [h, s, v]
            hsv_upper (list): Upper bound of HSV values [h, s, v]
            hsv_lower2 (list, optional): Second lower bound for colors that wrap around (like red)
            hsv_upper2 (list, optional): Second upper bound for colors that wrap around (like red)
        """
        self.color_ranges[name] = (hsv_lower, hsv_upper, hsv_lower2, hsv_upper2)
        return True
    
    def get_frame(self):
        """Get the current camera frame."""
        return self.frame
    
    def get_processed_frame(self):
        """Get the processed frame with detected objects highlighted."""
        return self.processed_frame
    
    def get_detected_objects(self):
        """
        Get information about detected objects.
        
        Returns:
            list: List of dictionaries containing information about detected objects
                 Each dictionary contains 'x', 'y', 'radius', 'area'
        """
        return self.detected_objects
    
    def _process_frames(self):
        """Process frames from the camera (internal method)."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
                
            self.frame = frame
            self.detected_objects = []
            
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get color range
            lower1, upper1, lower2, upper2 = self.color_ranges[self.current_color]
            
            # Create mask for the specified color
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            
            # If we have a second range (for colors like red that wrap around)
            if lower2 is not None and upper2 is not None:
                mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = mask1
            
            # Apply morphological operations to remove noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process contours
            processed_frame = frame.copy()
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter small contours
                if area < 500:
                    continue
                    
                # Find the center and radius of the contour
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                
                # Draw circle around the detected object
                cv2.circle(processed_frame, center, radius, (0, 255, 0), 2)
                cv2.circle(processed_frame, center, 5, (0, 0, 255), -1)
                
                # Add to detected objects
                self.detected_objects.append({
                    'x': x,
                    'y': y,
                    'radius': radius,
                    'area': area
                })
            
            # Draw color name and count
            cv2.putText(
                processed_frame, 
                f"Tracking: {self.current_color} ({len(self.detected_objects)} objects)", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.8, 
                (0, 165, 255), 
                2
            )
            
            self.processed_frame = processed_frame
            
            # Small delay to reduce CPU usage
            time.sleep(0.01)

# Simple PID controller for tracking
class PID:
    """
    A simple PID controller implementation.
    
    This class implements a Proportional-Integral-Derivative controller
    for smooth tracking of objects.
    """
    
    def __init__(self, p=0.2, i=0.0, d=0.0):
        """
        Initialize the PID controller.
        
        Args:
            p (float): Proportional gain
            i (float): Integral gain
            d (float): Derivative gain
        """
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.SetPoint = 0.0
        self.sample_time = 0.01
        self.current_time = time.time()
        self.last_time = self.current_time
        
        self.clear()
    
    def clear(self):
        """Clear the PID controller state."""
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
    
    def update(self, feedback_value):
        """
        Update the PID controller.
        
        Args:
            feedback_value (float): Current value from the system
            
        Returns:
            float: Controller output
        """
        error = self.SetPoint - feedback_value
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        
        if delta_time >= self.sample_time:
            # Calculate P term
            self.PTerm = self.Kp * error
            
            # Calculate I term
            self.ITerm += error * delta_time
            
            # Calculate D term
            delta_error = error - self.last_error
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            
            # Remember last time and error for next calculation
            self.last_time = self.current_time
            self.last_error = error
            
            # Calculate total output
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        
        return self.output

# Helper functions for color tracking applications
def display_image(image):
    """
    Display an image using OpenCV.
    
    Args:
        image: The image to display
    """
    cv2.imshow('Image', image)
    cv2.waitKey(1)

def close_all_windows():
    """Close all OpenCV windows."""
    cv2.destroyAllWindows()

def find_largest_object(objects):
    """
    Find the largest object in a list of detected objects.
    
    Args:
        objects (list): List of detected objects
        
    Returns:
        dict: The largest object, or None if no objects
    """
    if not objects:
        return None
    
    return max(objects, key=lambda obj: obj['area'])