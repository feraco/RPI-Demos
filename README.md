# 🚀 MentorPi Land Rover - ROS2 Control Documentation

The **MentorPi Land Rover** is an omni-wheel robotic platform that operates using ROS2. This documentation provides an overview of its movement functions and how to use them effectively.

## 📌 Movement Commands
The robot supports omni-directional movement using the following commands:

### 🔹 Forward Movement
Moves the robot forward at a specified speed for a given duration.
```python
move_forward(0.5, 3.0)  # Moves forward at 0.5 m/s for 3 seconds
```

### 🔹 Backward Movement
Moves the robot backward.
```python
move_backward(0.5, 3.0)  # Moves backward at 0.5 m/s for 3 seconds
```

### 🔹 Left Movement
Moves the robot to the left.
```python
move_left(0.5, 2.0)  # Moves left at 0.5 m/s for 2 seconds
```

### 🔹 Right Movement
Moves the robot to the right.
```python
move_right(0.5, 2.0)  # Moves right at 0.5 m/s for 2 seconds
```

### 🔹 Diagonal Movement
Moves the robot diagonally at a specified angle.
```python
move_in_direction(45, 0.5, 2.0)  # Moves at a 45° angle at 0.5 m/s for 2 seconds
```

### 🔹 Rotating Left (Counterclockwise)
Rotates the robot to the left (CCW).
```python
rotate_left(0.5, 2.0)  # Rotates CCW at 0.5 RPS for 2 seconds
```

### 🔹 Rotating Right (Clockwise)
Rotates the robot to the right (CW).
```python
rotate_right(0.5, 2.0)  # Rotates CW at 0.5 RPS for 2 seconds
```

### 🔹 Stopping All Motors
Immediately stops all motors.
```python
stop_all_motors()  # Stops all motor movement
```

---

## 📡 LiDAR Sensor Functions
The robot is equipped with a LiDAR sensor to detect obstacles and navigate autonomously.

### 🔹 Get LiDAR Readings
Retrieves and prints the current distances detected in front, back, left, and right directions.
```python
print_lidar_readings()  # Prints LiDAR distance values
```

### 🔹 Checking for Obstacles
Determines if an obstacle is within a critical distance.
```python
is_obstacle_near("front")  # Returns True if an obstacle is near the front
```

---

## 🔊 Buzzer Control
The buzzer can be activated for alerts.

### 🔹 Play Buzzer Sound
```python
play_buzzer(1000, 0.2, 0.2, 3)  # Plays a 1000Hz tone for 3 cycles
```

### 🔹 Stop Buzzer
```python
stop_buzzer()  # Stops the buzzer immediately
```

---

## 💡 LED Control
The robot is equipped with controllable RGB LEDs.

### 🔹 Set LED Color
```python
set_color(1, 255, 0, 0)  # Sets LED 1 to red
```

### 🔹 Set All LEDs to the Same Color
```python
set_all_colors(0, 255, 0)  # Sets all LEDs to green
```

### 🔹 Blink an LED
```python
blink(1, 0, 0, 255, 3, 0.5)  # Blinks LED 1 blue three times with 0.5s intervals
```

---

## 🚀 Example Autonomous Routine
Combines movement, LiDAR detection, and alerts into a simple autonomous behavior.
```python
if not is_obstacle_near("front"):
    move_forward(0.5, 2.0)
else:
    play_buzzer(1500, 0.1, 0.1, 2)
    move_backward(0.5, 1.0)
```

This script moves the robot forward unless an obstacle is detected, in which case it plays a buzzer sound and moves backward.

---

## 🎯 Conclusion
With these commands, the **MentorPi Land Rover** can execute precise movements, interact with its surroundings using LiDAR, and provide visual/audio feedback. Modify the examples to create custom robotic behaviors! 🚀

