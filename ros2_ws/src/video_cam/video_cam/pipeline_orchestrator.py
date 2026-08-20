#!/usr/bin/env python3
"""SIYI pipeline orchestrator — capture, index, download."""

import time
import numpy as np
import cv2
import rclpy.logging
from threading import Lock
from typing import Optional, Dict, Tuple
from .config import (
    CAPTURE_TIMEOUT_SECONDS,
    SD_POLL_INTERVAL,
    REQUIRED_DOWNLOAD_SPACE_MB,
    DEFAULT_RESOLUTION,
)
from .camera_interface import CameraInterface, CameraConnectionError
from .storage_manager import StorageManager


class PipelineOrchestrator:

    def __init__(self, camera: CameraInterface, storage: StorageManager,
                 rotate_180: bool = True, logger=None):
        self.camera = camera
        self.storage = storage
        self.logger = logger or rclpy.logging.get_logger('PipelineOrchestrator')

        self.capture_lock = Lock()

        self.photo_count: int = 0
        self.current_resolution: str = DEFAULT_RESOLUTION
        self.rotate_180 = rotate_180

        self.last_saved_path: Optional[str] = None

    def is_busy(self) -> bool:
        """Return True iff phases 1+2 (shutter + SD indexing) are in progress.

        Phase 3 (HTTP download) is deliberately NOT considered "busy" so
        that the next capture's shutter can overlap with a previous
        capture's download. Callers that want to gate a new trigger on
        "nothing going on at all" should not use this method.
        """
        return self.capture_lock.locked()
    
    def set_resolution(self, resolution: str):
        self.current_resolution = resolution
        self.logger.info(f"Resolution set to: {resolution}")
    
    def initialize_sd_card(self):
        """Initialize SD card state from camera"""
        self.camera.initialize_sd_card()

    def capture_and_index(self) -> Optional[Dict]:
        # Phases 1+2: fire shutter, poll SD card for the new file.
        if not self.capture_lock.acquire(blocking=False):
            self.logger.warn("capture_and_index: another capture is mid-shutter, dropping trigger")
            return None

        try:
            # Phase 1: trigger camera
            if not self._phase1_capture():
                self.logger.error("Phase 1 failed: capture command rejected")
                return None

            time.sleep(0.5)  # Brief wait for SD write

            # Phase 2: index SD card
            file_info = self._phase2_index()
            if not file_info:
                self.logger.error("Phase 2 failed: new image not found on SD card")
                return None

            return file_info
        except Exception as e:
            self.logger.error(f"capture_and_index exception: {e}")
            return None
        finally:
            self.capture_lock.release()

    def download_and_save(self,file_info: Dict,filename_override: Optional[str] = None,) -> Optional[Tuple[str, np.ndarray]]:
        """Phase 3: download bytes, decode, save atomically."""
        try:
            saved_path, img = self._phase3_download(
                file_info, filename_override=filename_override
            )
        except Exception as e:
            self.logger.error(f"download_and_save exception: {e}")
            return None

        if not saved_path or img is None:
            self.logger.error("Phase 3 failed: download or save error")
            return None

        self.last_saved_path = saved_path
        return saved_path, img

    def _phase1_capture(self) -> bool:
        # Phase 1: Trigger camera capture
        try:
            if not self.camera.send_capture_command(self.current_resolution):
                # Camera already logged why. Bailing here avoids the pointless
                # 15s Phase 2 poll for a file that was never written.
                self.logger.error("[Phase 1] Camera rejected the capture")
                return False
            self.photo_count += 1
            return True
        except CameraConnectionError as e:
            self.logger.error(f"[Phase 1] Failed: {e}")
            return False
    
    def _phase2_index(self, timeout: float = CAPTURE_TIMEOUT_SECONDS) -> Optional[Dict]:
        # Phase 2: Poll SD card for new image (simplified - no backoff)
        start_time = time.time()
        
        while (time.time() - start_time) < timeout: # 15s
            if new_file := self.camera.get_new_file(): # := is the walrus op. means assign and check at the same time.
                self.logger.info(f"Found new image: {new_file.get('name')}")
                return new_file
            time.sleep(SD_POLL_INTERVAL)
        
        self.logger.error(f"Timeout after {timeout}s")
        return None
    
    def _phase3_download(self, file_info: Dict, filename_override: Optional[str] = None,) -> tuple:
        """Phase 3: Download and save."""
        original_name = file_info.get('name', '')
        file_url = file_info.get('url', '')

        if not original_name or not file_url:
            return None, None

        save_name = filename_override or original_name

        # Download
        image_bytes = self._download_bytes(file_info)
        if not image_bytes:
            return None, None

        # Decode and verify
        img = self._decode_and_verify(image_bytes)
        if img is None:
            return None, None

        # Save under the target name. storage.save_image returns the full
        # absolute path on success or None on failure.
        saved_path = self.storage.save_image(save_name, img, self.current_resolution)
        if saved_path is None:
            return None, None

        # Note: the file was already claimed in downloaded_files by
        # _find_new_file (claim-at-find-time). No need to re-add here.
        return saved_path, img
    
    def _download_bytes(self, file_info: Dict) -> Optional[bytes]:
        """Download image bytes with disk space check"""
        file_size = file_info.get('size', 0)
        required_mb = max(REQUIRED_DOWNLOAD_SPACE_MB, (file_size * 2) / (1024 * 1024))
        
        if not self.storage.check_disk_space(required_mb):
            self.logger.error(f"Insufficient disk space (need {required_mb:.1f}MB)")
            return None
        
        return self.camera.download_image(file_info.get('url', ''))
    
    def _decode_and_verify(self, image_bytes: bytes) -> Optional[np.ndarray]:
        """Decode, verify, and rotate image (camera is mounted upside down)"""
        img = self.camera.decode_image(image_bytes)
        if img is None:
            return None
        # this is here because the camera is upside down all the time (might not need this if its gonna work properly during flight)
        if self.rotate_180:
            img = cv2.rotate(img, cv2.ROTATE_180)

        if not self.storage.verify_image(img, self.current_resolution):
            return None

        return img

    def get_stats(self) -> Dict:
        return {
            'photo_count': self.photo_count,
            'downloaded_files': self.camera.get_downloaded_count(),
            'current_directory': self.camera.current_photo_dir,
            'resolution': self.current_resolution,
        }
