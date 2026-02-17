#!/usr/bin/env python3
"""SIYI Image Storage Manager"""

import os
import cv2
import json
import shutil
import numpy as np
from PIL import Image as PILImage
from typing import Optional, Set, Tuple
from .config import (
    MIN_FILE_SIZE_BYTES,
    ATOMIC_WRITE_SUFFIX,
    WORKSPACE_SUBDIR,
    DOWNLOAD_SUBDIR,
    CAMERA_FEED_SUBDIR,
    MAPPING_SUBDIR,
    TRACKING_STATE_FILE,
    RESOLUTION_SPECS,
    MIN_FREE_SPACE_MB,
    JPEG_HEADER_BYTES,
    JPEG_FOOTER_BYTES,
)


class StorageError(Exception):
    """Raised when storage operation fails"""
    pass


class StorageManager:
    """Manages local image storage with atomic writes and verification"""
    
    def __init__(self, workspace_root: str, logger=None):
        """
        Initialize storage manager.
        
        Args:
            workspace_root: ROS2 workspace root directory
            logger: Optional logger (must have .info(), .warn(), .error() methods)
        """
        self.logger = logger
        
        # Directory structure
        video_cam_dir = os.path.join(workspace_root, WORKSPACE_SUBDIR)
        os.makedirs(video_cam_dir, exist_ok=True)
        
        self.download_dir = os.path.join(video_cam_dir, DOWNLOAD_SUBDIR)
        self.camera_feed_dir = os.path.join(video_cam_dir, CAMERA_FEED_SUBDIR)
        self.mapping_dir = os.path.join(video_cam_dir, MAPPING_SUBDIR)
        
        for directory in [self.download_dir, self.camera_feed_dir, self.mapping_dir]:
            os.makedirs(directory, exist_ok=True)
        
        self.tracking_file = os.path.join(self.download_dir, TRACKING_STATE_FILE)
        
        # Codec capabilities
        self.codec_capabilities = {
            'opencv_jpeg': False,
            'pil_available': False
        }
        self._detect_codec_support()
        
        self._log('info', "Storage manager initialized")
    
    def _log(self, level: str, message: str):
        """Internal logging wrapper"""
        if self.logger:
            log_func = getattr(self.logger, level, None)
            if log_func:
                log_func(message)
    
    def _detect_codec_support(self):
        """Detect available image codecs"""
        try:
            # Test OpenCV JPEG support
            test_img = np.zeros((10, 10, 3), dtype=np.uint8)
            test_path = os.path.join(self.download_dir, "opencv_test.jpg")
            
            try:
                success = cv2.imwrite(test_path, test_img)
                if success and os.path.exists(test_path):
                    self.codec_capabilities['opencv_jpeg'] = True
                    self._log('info', "OpenCV JPEG support: OK")
                    os.remove(test_path)
                else:
                    self._log('warn', "OpenCV JPEG support: WRITE FAILED")
            except Exception as e:
                self._log('warn', f"OpenCV JPEG support: {e}")
            
            # Test PIL availability
            try:
                self.codec_capabilities['pil_available'] = True
                self._log('info', "PIL/Pillow support: OK")
            except ImportError:
                self._log('warn', "PIL/Pillow not available")
            
            # Fail fast if no codecs available
            if not any(self.codec_capabilities.values()):
                raise RuntimeError("No JPEG codec available - cannot save images!")
            
            if not self.codec_capabilities['opencv_jpeg'] and self.codec_capabilities['pil_available']:
                self._log('warn', "Will use PIL/Pillow fallback for JPEG files")
                
        except RuntimeError:
            raise
        except Exception as e:
            self._log('warn', f"Codec check failed: {e}")
            # Assume OpenCV works as fallback
            self.codec_capabilities['opencv_jpeg'] = True
    
    def check_disk_space(self, required_mb: float = MIN_FREE_SPACE_MB) -> bool:
        """
        Check if sufficient disk space is available.
        
        Args:
            required_mb: Required free space in megabytes
            
        Returns:
            True if sufficient space available
        """
        try:
            stat = shutil.disk_usage(self.download_dir)
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
        """Get available disk space in megabytes"""
        try:
            stat = shutil.disk_usage(self.download_dir)
            return stat.free / (1024 * 1024)
        except Exception:
            return 0.0
    
    def save_image(self, filename: str, img: np.ndarray, 
                   resolution: str = '4K') -> Optional[str]:
        """
        Save image to download directory with atomic write.
        
        Args:
            filename: Target filename
            img: OpenCV image array (BGR)
            resolution: Image resolution for verification
            
        Returns:
            Full path to saved file or None if failed
        """
        try:
            filepath = os.path.join(self.download_dir, filename)
            
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
            tmp_path = filepath + ATOMIC_WRITE_SUFFIX
            
            # Determine if JPEG
            file_ext = os.path.splitext(filepath)[1].lower()
            is_jpeg = file_ext in ['.jpg', '.jpeg']
            
            success = False
            
            if is_jpeg:
                # Use best available JPEG codec
                if self.codec_capabilities.get('opencv_jpeg', False):
                    success = cv2.imwrite(tmp_path, img)
                elif self.codec_capabilities.get('pil_available', False):
                    # Convert BGR (OpenCV) to RGB (PIL)
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    pil_img = PILImage.fromarray(img_rgb)
                    pil_img.save(tmp_path, 'JPEG', quality=95)
                    success = True
                else:
                    raise RuntimeError("No JPEG codec available")
            else:
                # Non-JPEG file
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
        """
        Verify file exists and meets size requirements.
        
        Args:
            path: File path to verify
            resolution: Expected resolution for size threshold
            
        Returns:
            True if file valid
        """
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
        """
        Verify image is not corrupted or blank.
        
        Uses warnings instead of hard failures for brightness extremes.
        """
        try:
            if img is None:
                return False
            
            h, w = img.shape[:2]
            
            # Sanity check dimensions
            if h < 100 or w < 100:
                self._log('error', f"Image too small: {w}x{h}")
                return False
            
            # Check for extreme brightness (warning only)
            mean_val = np.mean(img)
            if mean_val < 5:
                self._log('warn', 
                    f"Image appears very dark (mean: {mean_val:.1f}) - "
                    "could be legitimate night shot")
            elif mean_val > 250:
                self._log('warn', 
                    f"Image appears very bright (mean: {mean_val:.1f}) - "
                    "could be snow/clouds or overexposure")
            
            return True
            
        except Exception as e:
            self._log('warn', f"Integrity check error: {e}")
            return True  # Don't fail on check errors
    
    def load_tracking_state(self) -> Tuple[Set[str], int]:
        """
        Load persistent tracking state from disk.
        
        Returns:
            Tuple of (downloaded_files_set, last_photo_count)
        """
        try:
            if os.path.exists(self.tracking_file):
                with open(self.tracking_file, 'r') as f:
                    data = json.load(f)
                
                # Restore downloaded files set (simple filenames)
                downloaded_files = set(data.get('downloaded_files', []))
                last_photo_count = data.get('last_photo_count', 0)
                
                self._log('info', 
                    f"Loaded tracking state: {len(downloaded_files)} files, "
                    f"last count: {last_photo_count}")
                
                return downloaded_files, last_photo_count
            else:
                self._log('info', "No tracking state file found")
                return set(), 0
                
        except Exception as e:
            self._log('warn', f"Could not load tracking state: {e}")
            return set(), 0
    
    def save_tracking_state(self, downloaded_files: Set[str], 
                           last_photo_count: int,
                           photo_count: int = 0,
                           max_tracked_files: int = 500):
        """
        Save persistent tracking state to disk with pruning.
        
        Args:
            downloaded_files: Set of filenames
            last_photo_count: Last known SD card photo count
            photo_count: Current capture count
            max_tracked_files: Maximum files to keep in tracking
        """
        try:
            # Prune old entries
            if len(downloaded_files) > max_tracked_files:
                old_count = len(downloaded_files)
                downloaded_files = set(list(downloaded_files)[:max_tracked_files])
                self._log('info', f"Pruned tracking state: {old_count} → {max_tracked_files}")
            
            import time
            data = {
                'downloaded_files': list(downloaded_files),
                'last_photo_count': last_photo_count,
                'last_operation_time': time.strftime("%Y-%m-%d %H:%M:%S"),
                'photo_count': photo_count
            }
            
            # Atomic write
            tmp_file = self.tracking_file + ATOMIC_WRITE_SUFFIX
            with open(tmp_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            os.replace(tmp_file, self.tracking_file)
            
        except Exception as e:
            self._log('warn', f"Could not save tracking state: {e}")
    
    def get_directories(self) -> Tuple[str, str, str]:
        """
        Get storage directory paths.
        
        Returns:
            Tuple of (download_dir, camera_feed_dir, mapping_dir)
        """
        return self.download_dir, self.camera_feed_dir, self.mapping_dir
