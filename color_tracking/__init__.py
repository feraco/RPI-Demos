"""
Color Tracking Module

This module provides functionality for tracking colored objects using a camera.
"""

from .color_tracker import ColorTracker, PID, find_largest_object, display_image, close_all_windows

__all__ = ['ColorTracker', 'PID', 'find_largest_object', 'display_image', 'close_all_windows']