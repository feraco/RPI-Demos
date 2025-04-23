# Color Tracking Module

Simple module for tracking colored objects using a camera.

## Features

- Track objects of predefined colors (red, green, blue, yellow)
- Define custom color ranges for tracking
- Get information about detected objects (position, size, area)
- PID controller for object following

## Usage

```python
from color_tracking.color_tracker import ColorTracker

# Create a tracker
tracker = ColorTracker(camera_index=0)

# Start the tracker
tracker.start()

# Process a frame
frame = tracker.process_frame()

# Set the color to track
tracker.set_color('red')

# Get detected objects
objects = tracker.get_detected_objects()

# Stop the tracker
tracker.stop()
```

## Tutorial

See `Color_Tracking_Tutorial.ipynb` for a step-by-step guide.

## Requirements

- OpenCV (cv2)
- NumPy

## Applications

- Object following robots
- Color-based sorting systems
- Interactive games
- Visual feedback systems