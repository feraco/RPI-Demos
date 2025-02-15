import numpy as np

def is_obstacle_ahead(node, threshold=0.5):
    """Returns True if an obstacle is within `threshold` meters in front of the robot."""
    return node.min_distance < threshold

def is_clear_path_forward(node, min_safe_distance=0.8):
    """Returns True if there are no obstacles within `min_safe_distance` meters ahead."""
    return node.min_distance >= min_safe_distance

def should_turn_left(node):
    """Returns True if the left side is more open than the right side."""
    return get_side_distance(node, "left") > get_side_distance(node, "right")

def should_turn_right(node):
    """Returns True if the right side is more open than the left side."""
    return get_side_distance(node, "right") > get_side_distance(node, "left")

def should_stop_moving(node, stop_threshold=0.4):
    """Stops the robot if obstacles are detected on all sides within `stop_threshold` meters."""
    left = get_side_distance(node, "left")
    right = get_side_distance(node, "right")

    return node.min_distance < stop_threshold and left < stop_threshold and right < stop_threshold

def get_best_turn_direction(node):
    """Returns 'left' or 'right' based on which direction has more open space."""
    return "left" if get_side_distance(node, "left") > get_side_distance(node, "right") else "right"

def get_side_distance(node, direction):
    """Returns the LiDAR distance for the given direction ('left' or 'right')."""
    if len(node.lidar_ranges) == 0:
        return float('inf')

    left_index = int(len(node.lidar_ranges) * 3 / 4)  # Approx. 270 degrees
    right_index = int(len(node.lidar_ranges) / 4)  # Approx. 90 degrees

    if direction == "left":
        return node.lidar_ranges[left_index] if np.isfinite(node.lidar_ranges[left_index]) else float('inf')

    elif direction == "right":
        return node.lidar_ranges[right_index] if np.isfinite(node.lidar_ranges[right_index]) else float('inf')

    return float('inf')  # Default return if direction is invalid
