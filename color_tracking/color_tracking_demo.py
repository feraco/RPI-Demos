#!/usr/bin/env python3
# encoding: utf-8
"""
Color Tracking Demo

This script demonstrates how to use the color tracking module
to detect and track colored objects using a camera.
"""

import time
import cv2
import sys
import os

# Add parent directory to the Python path
sys.path.insert(0, os.path.abspath('..'))

# Import the color tracking module
from color_tracking.color_tracker import ColorTracker, find_largest_object

def main():
    """Main function to demonstrate color tracking."""
    print("Color Tracking Demo")
    print("-------------------")
    print("Press 'r' for red, 'g' for green, 'b' for blue, 'y' for yellow")
    print("Press 'q' to quit")
    
    # Create a color tracker
    tracker = ColorTracker(camera_index=0)
    
    try:
        # Start the tracker
        tracker.start()
        print("Tracker started")
        
        # Main loop
        while True:
            # Get the processed frame
            frame = tracker.get_processed_frame()
            
            if frame is not None:
                # Display the frame
                cv2.imshow('Color Tracking Demo', frame)
                
                # Get information about detected objects
                objects = tracker.get_detected_objects()
                largest = find_largest_object(objects)
                
                # Display information about the largest object
                if largest:
                    print(f"\rTracking: {tracker.current_color} | "
                          f"Position: ({largest['x']:.1f}, {largest['y']:.1f}) | "
                          f"Size: {largest['radius']:.1f} pixels", end='')
                else:
                    print(f"\rTracking: {tracker.current_color} | No objects detected", end='')
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            
            # Change color based on key press
            if key == ord('r'):
                tracker.set_color('red')
                print("\nSwitched to tracking red")
            elif key == ord('g'):
                tracker.set_color('green')
                print("\nSwitched to tracking green")
            elif key == ord('b'):
                tracker.set_color('blue')
                print("\nSwitched to tracking blue")
            elif key == ord('y'):
                tracker.set_color('yellow')
                print("\nSwitched to tracking yellow")
            elif key == ord('q'):
                break
            
            # Small delay to reduce CPU usage
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Clean up
        tracker.stop()
        cv2.destroyAllWindows()
        print("\nTracker stopped and windows closed")

if __name__ == "__main__":
    main()