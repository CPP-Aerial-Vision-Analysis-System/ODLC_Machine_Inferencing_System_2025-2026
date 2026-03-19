#!/usr/bin/env python3
"""SIYI image storage manager."""

import os
import cv2
import json
import time
import shutil
import numpy as np
from typing import Optional, Set, Tuple
from .config import (
    MIN_FILE_SIZE_BYTES,
    ATOMIC_WRITE_SUFFIX,
    WORKSPACE_SUBDIR,
    MAPPING_SUBDIR,
    TRACKING_STATE_FILE, # removed for now 
    RESOLUTION_SPECS,
    MIN_FREE_SPACE_MB,
    JPEG_HEADER_BYTES,
    JPEG_FOOTER_BYTES,
)


class StorageError(Exception):
    """Raised when storage operation fails"""
    pass


class StorageManager:
    
    def __init__(self, workspace_root: str, logger=None):
        # logger: Optional logger (must have .info(), .warn(), .error() methods)
        self.logger = logger

        # Save images into ros2_ws/video_cam/mapping_photos
        video_cam_dir = os.path.join(workspace_root)
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Single directory for all images (capture, download, mapping)
        self.mapping_dir = os.path.join(video_cam_dir, MAPPING_SUBDIR)
        os.makedirs(self.mapping_dir, exist_ok=True)
        
        # Prevents duplicate downloads
        # self.tracking_file = os.path.join(self.mapping_dir, TRACKING_STATE_FILE)
        
        self._log('info', "Storage manager initialized")
    
    def _log(self, level: str, message: str):
        if self.logger:
            log_func = getattr(self.logger, level, None)
            if log_func:
                log_func(message)
    
    def check_disk_space(self, required_mb: float = MIN_FREE_SPACE_MB) -> bool:
        try:
            stat = shutil.disk_usage(self.mapping_dir)
            free_mb = stat.free / (1024 * 1024)
            
            if free_mb < required_mb:
                self._log('error', 
                    f"Disk space critical: {free_mb:.1f}MB free (need {required_mb:.1f}MB)")
                return False
            
            return True
        except Exception as e:
            self._log('warn', f"Could not check disk space: {e}")
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
                self._log('error', f"Failed to write: {filepath}")
                return None
            
            # Verify saved file
            if not self.verify_file(filepath, resolution):
                self._log('error', f"File verification failed: {filepath}")
                return None
            
            self._log('info', f"Saved: {filepath}")
            return filepath
            
        except Exception as e:
            self._log('error', f"Save error: {e}")
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
            self._log('error', f"Atomic write error: {e}")
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
                self._log('warn', 
                    f"File size {size} bytes below minimum {min_bytes} bytes for {resolution}")
                return False
            
            return True
            
        except Exception as e:
            self._log('warn', f"File verification error: {e}")
            return False
   
    # def save_tracking_state(self, downloaded_files: Set[str], last_photo_count: int, photo_count: int = 0, max_tracked_files: int = 500):
    #     # This makes sure we dont redownload photos after we crash, restart, mission pause, etc..
    #     try:
    #         # Prune old entries
    #         if len(downloaded_files) > max_tracked_files:
    #             old_count = len(downloaded_files)
    #             downloaded_files = set(list(downloaded_files)[:max_tracked_files])
    #             self._log('info', f"Pruned tracking state: {old_count} → {max_tracked_files}")
            
    #         data = {
    #             'downloaded_files': list(downloaded_files),
    #             'last_photo_count': last_photo_count,
    #             'last_operation_time': time.strftime("%Y-%m-%d %H:%M:%S"),
    #             'photo_count': photo_count
    #         }
            
    #         # Atomic write
    #         tmp_file = self.tracking_file + ATOMIC_WRITE_SUFFIX
    #         with open(tmp_file, 'w') as f:
    #             json.dump(data, f, indent=2)
            
    #         os.replace(tmp_file, self.tracking_file)
            
    #     except Exception as e:
    #         self._log('warn', f"Could not save tracking state: {e}")
    
    def get_mapping_dir(self) -> str:
        return self.mapping_dir

    # The methods below may seem useless, but sometimes camera tweaks(cause of bandwith drops for example) and returns a junk data, this is neded to prevent it
    def verify_image_dimensions(self, img: np.ndarray, resolution: str = '4K') -> bool:
        """Verify image meets minimum dimension requirements"""
        if img is None:
            return False
        
        try:
            h, w = img.shape[:2]
            
            specs = RESOLUTION_SPECS.get(resolution)
            if specs:
                if w >= specs['min_width'] and h >= specs['min_height']:
                    return True
                else:
                    self._log('warn', 
                        f"Dimensions {w}x{h} below {resolution} threshold "
                        f"({specs['min_width']}x{specs['min_height']})")
                    return False
            
            # Fallback: just check that it's not too small
            return w > 640 and h > 480
            
        except Exception:
            return False
   
    def verify_image_integrity(self, img: np.ndarray) -> bool:
        """Verify image is not corrupted (minimum dimension check)."""
        try:
            if img is None:
                return False
            h, w = img.shape[:2]
            if h < 100 or w < 100:
                self._log('error', f"Image too small: {w}x{h}")
                return False
            return True
        except Exception as e:
            self._log('warn', f"Integrity check error: {e}")
            return True
    # def load_tracking_state(self) -> Tuple[Set[str], int]:
    #     """
    #     Load persistent tracking state from disk.
        
    #     Returns:
    #         Tuple of (downloaded_files_set, last_photo_count)
    #     """
    #     try:
    #         if os.path.exists(self.tracking_file):
    #             with open(self.tracking_file, 'r') as f:
    #                 data = json.load(f)
                
    #             # Restore downloaded files set (simple filenames)
    #             downloaded_files = set(data.get('downloaded_files', []))
    #             last_photo_count = data.get('last_photo_count', 0)
                
    #             self._log('info', 
    #                 f"Loaded tracking state: {len(downloaded_files)} files, "
    #                 f"last count: {last_photo_count}")
                
    #             return downloaded_files, last_photo_count
    #         else:
    #             self._log('info', "No tracking state file found")
    #             return set(), 0
                
    #     except Exception as e:
    #         self._log('warn', f"Could not load tracking state: {e}")
    #         return set(), 0
    