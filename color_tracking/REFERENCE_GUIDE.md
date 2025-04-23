# Color Tracking and Reference Functions Guide

This guide provides a comprehensive reference for the color tracking module and related functions from other modules that can be used together for robotics projects.

## Table of Contents

1. [Color Tracking Module](#color-tracking-module)
2. [Image Capture Functions](#image-capture-functions)
3. [Joystick Control Functions](#joystick-control-functions)
4. [LIDAR Functions](#lidar-functions)
5. [Robot Control Functions](#robot-control-functions)
6. [QR Code Functions](#qr-code-functions)
7. [Integration Examples](#integration-examples)

## Color Tracking Module

### ColorTracker Class

```python
from color_tracking.color_tracker import ColorTracker

# Create a tracker
tracker = ColorTracker(camera_index=0)
```

#### Main Methods

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `start()` | Start the camera and initialize tracking | None | `True` on success |
| `stop()` | Stop tracking and release the camera | None | `True` on success |
| `process_frame()` | Process a single frame from the camera | None | Processed frame (image) |
| `set_color(color_name)` | Set the color to track | `color_name`: string ('red', 'green', 'blue', 'yellow') | `True` if color exists |
| `add_custom_color(name, hsv_lower, hsv_upper, hsv_lower2=None, hsv_upper2=None)` | Add a custom color range | `name`: string, `hsv_lower`/`hsv_upper`: [h,s,v] lists | `True` on success |
| `get_frame()` | Get the current camera frame | None | Raw frame (image) |
| `get_processed_frame()` | Get the processed frame | None | Processed frame (image) |
| `get_detected_objects()` | Get information about detected objects | None | List of object dictionaries |

#### PID Controller

```python
from color_tracking.color_tracker import PID

# Create a PID controller
pid = PID(p=0.2, i=0.01, d=0.05)
```

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `clear()` | Reset the PID controller | None | None |
| `update(feedback_value)` | Update the PID controller | `feedback_value`: current value | Controller output |

#### Helper Functions

| Function | Description | Parameters | Returns |
|----------|-------------|------------|---------|
| `find_largest_object(objects)` | Find the largest object | `objects`: list of detected objects | Largest object or None |
| `display_image(image)` | Display an image using OpenCV | `image`: image to display | None |
| `close_all_windows()` | Close all OpenCV windows | None | None |

## Image Capture Functions

```python
from color_tracking.image_capture import capture_image, save_image, display_image
```

| Function | Description | Parameters | Returns |
|----------|-------------|------------|---------|
| `capture_image(camera_index=0)` | Capture an image from camera | `camera_index`: camera to use | Image (numpy array) |
| `save_image(image, filename)` | Save an image to a file | `image`: image to save, `filename`: path | `True` on success |
| `display_image(image, window_name='Image')` | Display an image | `image`: image to display, `window_name`: title | None |

## Joystick Control Functions

```python
from color_tracking.joystick_control import JoystickController
```

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `JoystickController(joystick_id=0)` | Create a joystick controller | `joystick_id`: joystick to use | Controller object |
| `start()` | Start the joystick controller | None | None |
| `stop()` | Stop the joystick controller | None | None |
| `get_axis(axis_id)` | Get axis value | `axis_id`: axis to read | Float (-1.0 to 1.0) |
| `get_button(button_id)` | Get button state | `button_id`: button to read | Boolean |
| `set_deadzone(value)` | Set joystick deadzone | `value`: deadzone (0.0-1.0) | None |

## LIDAR Functions

```python
from color_tracking.lidar_controller import LidarController
from color_tracking.lidar_utils import filter_scan, find_closest_point
```

### LidarController Class

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `LidarController(port='/dev/ttyUSB0')` | Create a LIDAR controller | `port`: serial port | Controller object |
| `start_scan()` | Start LIDAR scanning | None | None |
| `stop_scan()` | Stop LIDAR scanning | None | None |
| `get_scan()` | Get current scan data | None | List of distance measurements |

### LIDAR Utility Functions

| Function | Description | Parameters | Returns |
|----------|-------------|------------|---------|
| `filter_scan(scan_data, min_dist=0.1, max_dist=10.0)` | Filter scan data | `scan_data`: raw scan, `min_dist`/`max_dist`: range | Filtered scan data |
| `find_closest_point(scan_data)` | Find closest point in scan | `scan_data`: scan data | (angle, distance) |
| `detect_obstacles(scan_data, threshold=0.5)` | Detect obstacles | `scan_data`: scan data, `threshold`: distance | List of obstacles |

## Robot Control Functions

```python
from color_tracking.omni_robot_controller import OmniRobotController
from color_tracking.OmniWheelControl import OmniWheelRobot
```

### OmniRobotController Class

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `OmniRobotController()` | Create a robot controller | None | Controller object |
| `move(x_vel, y_vel, angular_vel)` | Move the robot | `x_vel`, `y_vel`, `angular_vel`: velocities | None |
| `stop()` | Stop the robot | None | None |
| `set_max_speed(speed)` | Set maximum speed | `speed`: maximum speed | None |

### OmniWheelRobot Class

| Method | Description | Parameters | Returns |
|--------|-------------|------------|---------|
| `OmniWheelRobot(wheel_radius, robot_radius)` | Create an omni wheel robot | `wheel_radius`, `robot_radius`: dimensions | Robot object |
| `set_wheel_velocities(v1, v2, v3, v4)` | Set wheel velocities | `v1`-`v4`: wheel velocities | None |
| `move(x_vel, y_vel, angular_vel)` | Move the robot | `x_vel`, `y_vel`, `angular_vel`: velocities | None |
| `stop()` | Stop the robot | None | None |

## QR Code Functions

```python
from color_tracking.qr_code_tools import detect_qr_codes, decode_qr_code
from color_tracking.qrcode_creater import create_qr_code
from color_tracking.qrcode_detecter import detect_qr_code
```

| Function | Description | Parameters | Returns |
|----------|-------------|------------|---------|
| `detect_qr_codes(image)` | Detect QR codes in image | `image`: input image | List of QR code objects |
| `decode_qr_code(qr_code)` | Decode QR code data | `qr_code`: QR code object | Decoded string |
| `create_qr_code(data, filename)` | Create a QR code image | `data`: string to encode, `filename`: output path | `True` on success |
| `detect_qr_code(image)` | Detect and decode QR code | `image`: input image | (data, position) |

## Integration Examples

### Color Tracking with Robot Control

```python
from color_tracking.color_tracker import ColorTracker, find_largest_object, PID
from color_tracking.omni_robot_controller import OmniRobotController

# Initialize components
tracker = ColorTracker(camera_index=0)
robot = OmniRobotController()

# Set up PID controllers
pid_x = PID(p=0.1, i=0.01, d=0.05)
pid_y = PID(p=0.1, i=0.01, d=0.05)

# Start tracking
tracker.start()
tracker.set_color('red')

# Main loop
while True:
    # Process a frame
    frame = tracker.process_frame()
    
    # Get the largest object
    objects = tracker.get_detected_objects()
    largest = find_largest_object(objects)
    
    if largest:
        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Set PID targets to center of frame
        pid_x.SetPoint = center_x
        pid_y.SetPoint = center_y
        
        # Update PID controllers
        pid_x.update(largest['x'])
        pid_y.update(largest['y'])
        
        # Calculate movement values
        move_x = -pid_x.output * 0.01  # Scale for robot control
        move_y = -pid_y.output * 0.01  # Scale for robot control
        
        # Move the robot
        robot.move(move_x, move_y, 0)
    else:
        # No object detected, stop the robot
        robot.stop()
```

### LIDAR Obstacle Avoidance with Color Tracking

```python
from color_tracking.color_tracker import ColorTracker, find_largest_object
from color_tracking.lidar_controller import LidarController
from color_tracking.lidar_utils import find_closest_point
from color_tracking.omni_robot_controller import OmniRobotController

# Initialize components
tracker = ColorTracker(camera_index=0)
lidar = LidarController()
robot = OmniRobotController()

# Start components
tracker.start()
lidar.start_scan()
tracker.set_color('green')

# Main loop
while True:
    # Process a frame
    frame = tracker.process_frame()
    
    # Get LIDAR scan
    scan = lidar.get_scan()
    closest_point = find_closest_point(scan)
    
    # Check if obstacle is too close
    if closest_point[1] < 0.5:  # If closer than 0.5 meters
        # Obstacle avoidance mode
        robot.stop()
        # Calculate avoidance direction based on obstacle position
        avoid_angle = closest_point[0]
        # Move away from obstacle
        robot.move(math.cos(avoid_angle + math.pi) * 0.2, 
                  math.sin(avoid_angle + math.pi) * 0.2, 
                  0)
    else:
        # Normal tracking mode
        objects = tracker.get_detected_objects()
        largest = find_largest_object(objects)
        
        if largest:
            # Calculate direction to object
            height, width = frame.shape[:2]
            center_x = width // 2
            center_y = height // 2
            
            # Simple proportional control
            move_x = (center_x - largest['x']) * 0.001
            move_y = (center_y - largest['y']) * 0.001
            
            # Move the robot
            robot.move(move_x, move_y, 0)
        else:
            # No object detected, stop the robot
            robot.stop()
```

### QR Code Navigation with Color Tracking

```python
from color_tracking.color_tracker import ColorTracker
from color_tracking.qr_code_tools import detect_qr_codes, decode_qr_code
from color_tracking.omni_robot_controller import OmniRobotController

# Initialize components
tracker = ColorTracker(camera_index=0)
robot = OmniRobotController()

# Start tracking
tracker.start()

# Main loop
while True:
    # Process a frame
    frame = tracker.process_frame()
    
    # Look for QR codes
    qr_codes = detect_qr_codes(frame)
    
    if qr_codes:
        # QR code found, decode it
        qr_data = decode_qr_code(qr_codes[0])
        
        # Parse navigation commands from QR code
        # Example format: "MOVE:X=0.5,Y=-0.2,R=0.1"
        if qr_data.startswith("MOVE:"):
            params = qr_data[5:].split(',')
            x_vel = float(params[0].split('=')[1])
            y_vel = float(params[1].split('=')[1])
            r_vel = float(params[2].split('=')[1])
            
            # Execute movement command
            robot.move(x_vel, y_vel, r_vel)
    else:
        # No QR code found, use color tracking
        tracker.set_color('blue')
        objects = tracker.get_detected_objects()
        
        if objects:
            # Follow the first detected object
            obj = objects[0]
            height, width = frame.shape[:2]
            
            # Simple proportional control
            move_x = (width/2 - obj['x']) * 0.001
            move_y = (height/2 - obj['y']) * 0.001
            
            # Move the robot
            robot.move(move_x, move_y, 0)
        else:
            # No object detected, stop the robot
            robot.stop()
```

This reference guide provides a comprehensive overview of the available functions and how they can be integrated for various robotics applications. Use these examples as starting points for your own projects.