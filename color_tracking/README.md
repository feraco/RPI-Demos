# Color Tracking Module

This module provides functionality for tracking colored objects using a camera. It includes classes and functions for detecting and tracking objects of specific colors.

## Features

- Track objects of predefined colors (red, green, blue, yellow)
- Define custom color ranges for tracking
- Get information about detected objects (position, size, area)
- Simple PID controller for smooth tracking
- Helper functions for processing detected objects

## Usage

The main class is `ColorTracker`, which handles the camera feed and color detection:

```python
from color_tracking.color_tracker import ColorTracker

# Create a tracker using the default camera
tracker = ColorTracker(camera_index=0)

# Start the tracker
tracker.start()

# Set the color to track
tracker.set_color('red')

# Get information about detected objects
objects = tracker.get_detected_objects()

# When done, stop the tracker to release the camera
tracker.stop()
```

## Tutorial

Check out the `Color_Tracking_Tutorial.ipynb` notebook for a step-by-step guide on using the color tracking module.

## Requirements

- OpenCV (cv2)
- NumPy
- Matplotlib (for visualization in the notebook)

## Applications

This module can be used for various robotics applications:
- Object following robots
- Color-based sorting systems
- Interactive games and installations
- Visual feedback systems