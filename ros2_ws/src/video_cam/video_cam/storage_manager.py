#!/usr/bin/env python3
"""SIYI image storage manager."""

import os
import cv2
import json
import time
import shutil
import numpy as np
import rclpy.logging
from typing import Optional, Set, Tuple

def get_ros2_ws_directory() -> str:
    """Helper method to determine the ros2_ws/src directory dynamically.

    Walks up from this file looking for a directory containing both
    `install/` and `src/` (the standard ROS2 workspace layout). Falls
    back to the user's home directory so the package still functions on
    machines whose workspace isn't laid out the expected way.
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))

    search_dir = current_dir
    for _ in range(10):  # Limit search depth
        if (os.path.isdir(os.path.join(search_dir, "install"))
                and os.path.isdir(os.path.join(search_dir, "src"))):
            return os.path.join(search_dir, "src")
        parent = os.path.dirname(search_dir)
        if parent == search_dir:
            break
        search_dir = parent

    # Fallback: user home, so we don't write to a hard-coded absolute path
    # that only exists on one machine.
    return os.path.expanduser("~")

from .config import (
    MIN_FILE_SIZE_BYTES,
    ATOMIC_WRITE_SUFFIX,
    MAPPING_SUBDIR,
    RESOLUTION_SPECS,
    MIN_FREE_SPACE_MB,
)


class StorageManager:
    
    def __init__(self, workspace_root: str, logger=None):
        self.logger = logger or rclpy.logging.get_logger('StorageManager')

        # Save images into ros2_ws/video_cam/mapping_photos
        video_cam_dir = os.path.join(workspace_root)
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Single directory for all images (capture, download, mapping)
        self.mapping_dir = os.path.join(video_cam_dir, MAPPING_SUBDIR)
        os.makedirs(self.mapping_dir, exist_ok=True)
        
    def check_disk_space(self, required_mb: float = MIN_FREE_SPACE_MB) -> bool:
        try:
            stat = shutil.disk_usage(self.mapping_dir)
            free_mb = stat.free / (1024 * 1024)
            
            if free_mb < required_mb:
                self.logger.error(
                    f"Disk space critical: {free_mb:.1f}MB free (need {required_mb:.1f}MB)")
                return False
            
            return True
        except Exception as e:
            self.logger.warn(f"Could not check disk space: {e}")
            return True  # Assume OK if check fails
    
    def get_free_space_mb(self) -> float:
        try:
            stat = shutil.disk_usage(self.mapping_dir)
            return stat.free / (1024 * 1024)
        except Exception:
            return 0.0
    
    def save_image(self, filename: str, img: np.ndarray, 
                   resolution: str = '4K') -> Optional[str]:
        # Save image to mapping directory with atomic write.
        try:
            filepath = os.path.join(self.mapping_dir, filename)
            
            # Atomic write
            if not self._atomic_write(filepath, img):
                self.logger.error(f"Failed to write: {filepath}")
                return None
            
            # Verify saved file
            if not self.verify_file(filepath, resolution):
                self.logger.error(f"File verification failed: {filepath}")
                return None
            
            self.logger.info(f"Saved: {filepath}")
            return filepath
            
        except Exception as e:
            self.logger.error(f"Save error: {e}")
            return None
    
    def _atomic_write(self, filepath: str, img: np.ndarray) -> bool:
        """Write image file atomically (temp file + rename)"""
        tmp_path = None
        try:
            # Preserve extension so OpenCV can determine the encoder
            # e.g. IMG_0079.jpg -> IMG_0079.tmp.jpg (not IMG_0079.jpg.tmp)
            base, ext = os.path.splitext(filepath)
            tmp_path = base + ATOMIC_WRITE_SUFFIX + ext
            
            # Write image using OpenCV
            success = cv2.imwrite(tmp_path, img)
            
            if not success:
                return False
            
            # Verify temp file before committing
            if not os.path.exists(tmp_path) or os.path.getsize(tmp_path) < MIN_FILE_SIZE_BYTES:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
                return False
            
            # Atomic rename
            os.replace(tmp_path, filepath)
            return True
            
        except Exception as e:
            self.logger.error(f"Atomic write error: {e}")
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except:
                    pass
            return False
    
    def verify_file(self, path: str, resolution: str = '4K') -> bool:
        # Verify file exists and meets size requirements.
        
        try:
            if not os.path.exists(path):
                return False
            
            size = os.path.getsize(path)
            
            # Get resolution-specific minimum
            specs = RESOLUTION_SPECS.get(resolution, {})
            min_bytes = specs.get('min_file_size', MIN_FILE_SIZE_BYTES)
            
            if size < min_bytes:
                self.logger.warn(
                    f"File size {size} bytes below minimum {min_bytes} bytes for {resolution}")
                return False
            
            return True
            
        except Exception as e:
            self.logger.warn(f"File verification error: {e}")
            return False
   
    def get_mapping_dir(self) -> str:
        return self.mapping_dir

    # The methods below may seem useless, but sometimes camera tweaks(cause of bandwith drops for example) and returns a junk data, this is neded to prevent it
    def verify_image(self, img: np.ndarray, resolution: str = '4K') -> bool:
        """Verify image is not corrupted and meets minimum dimension requirements."""
        if img is None:
            return False

        try:
            h, w = img.shape[:2]

            if h < 100 or w < 100:
                self.logger.error(f"Image too small: {w}x{h}")
                return False

            specs = RESOLUTION_SPECS.get(resolution)
            if specs:
                if w < specs['min_width'] or h < specs['min_height']:
                    self.logger.warn(
                        f"Dimensions {w}x{h} below {resolution} threshold "
                        f"({specs['min_width']}x{specs['min_height']})")
                    return False

            return True

        except Exception as e:
            self.logger.warn(f"Image verification error: {e}")
            return False