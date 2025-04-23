#!/usr/bin/env python3
# encoding: utf-8
"""
Color Tracking Module

Simple module for tracking colored objects using a camera.
"""

import cv2
import numpy as np
import time

class ColorTracker:
    """Simple class for tracking colored objects in a video stream."""
    
    def __init__(self, camera_index=0):
        """Initialize the ColorTracker with a camera."""
        self.camera_index = camera_index
        self.cap = None
        self.running = False
        self.frame = None
        self.processed_frame = None
        
        # Define color ranges in HSV format
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),
            'green': ([35, 100, 100], [85, 255, 255], None, None),
            'blue': ([100, 100, 100], [140, 255, 255], None, None),
            'yellow': ([20, 100, 100], [35, 255, 255], None, None)
        }
        self.current_color = 'red'
        self.detected_objects = []
    
    def start(self):
        """Start the camera and initialize tracking."""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")
        self.running = True
        return True
    
    def stop(self):
        """Stop tracking and release the camera."""
        self.running = False
        if self.cap:
            self.cap.release()
        return True
    
    def set_color(self, color_name):
        """Set the color to track."""
        if color_name in self.color_ranges:
            self.current_color = color_name
            return True
        return False
    
    def add_custom_color(self, name, hsv_lower, hsv_upper, hsv_lower2=None, hsv_upper2=None):
        """Add a custom color range to track."""
        self.color_ranges[name] = (hsv_lower, hsv_upper, hsv_lower2, hsv_upper2)
        return True
    
    def process_frame(self):
        """Process a single frame from the camera."""
        if not self.running or not self.cap:
            return None
            
        ret, frame = self.cap.read()
        if not ret:
            return None
            
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
        return processed_frame
    
    def get_frame(self):
        """Get the current camera frame."""
        return self.frame
    
    def get_processed_frame(self):
        """Get the processed frame with detected objects highlighted."""
        return self.processed_frame
    
    def get_detected_objects(self):
        """Get information about detected objects."""
        return self.detected_objects

class PID:
    """Simple PID controller for tracking."""
    
    def __init__(self, p=0.2, i=0.0, d=0.0):
        """Initialize the PID controller."""
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.SetPoint = 0.0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
    
    def clear(self):
        """Reset the PID controller."""
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
    
    def update(self, feedback_value):
        """Update the PID controller with a new value."""
        error = self.SetPoint - feedback_value
        
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        
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

# Helper functions
def find_largest_object(objects):
    """Find the largest object in a list of detected objects."""
    if not objects:
        return None
    
    return max(objects, key=lambda obj: obj['area'])

def display_image(image):
    """Display an image using OpenCV."""
    cv2.imshow('Image', image)
    cv2.waitKey(1)

def close_all_windows():
    """Close all OpenCV windows."""
    cv2.destroyAllWindows()