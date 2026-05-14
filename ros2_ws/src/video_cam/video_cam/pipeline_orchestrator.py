#!/usr/bin/env python3
"""SIYI pipeline orchestrator — capture, index, download."""

import time
import numpy as np
import cv2
import rclpy.logging
from threading import Lock
from typing import Optional, Set, Dict, Tuple
from .config import (
    CaptureState,
    CAPTURE_TIMEOUT_SECONDS,
    MAX_PIPELINE_DURATION,
    SD_POLL_INTERVAL,
    REQUIRED_DOWNLOAD_SPACE_MB,
)
from .camera_interface import CameraInterface, CameraConnectionError
from .storage_manager import StorageManager


class PipelineError(Exception): 
    # Raised when pipeline execution fails. This gets assigned as {e} when we get errors, we give it some value then send it to execute
    pass

class PipelineOrchestrator:
    
    def __init__(self, camera: CameraInterface, storage: StorageManager, logger=None):
        self.camera = camera
        self.storage = storage
        self.logger = logger or rclpy.logging.get_logger('PipelineOrchestrator')

        self.pipeline_state = CaptureState.IDLE
        self.state_lock = Lock()
        self.capture_lock = Lock()

        # Simple filename-only tracking
        self.downloaded_files: Set[str] = set()
        self.download_lock = Lock()
        
        self.current_photo_dir: Optional[str] = None
        self.last_photo_count: int = 0
        self.photo_count: int = 0
        self.current_resolution: str = '4K'

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
        self.logger.info("Initializing SD card...")
        
        try:
            directories = self.camera.get_directories()
            # print (directories)
            if directories:
                self.current_photo_dir = directories[-1]['path']
                if self.last_photo_count == 0:
                    count = self.camera.get_media_count(self.current_photo_dir)
                    if count is not None:
                        self.last_photo_count = count
                    self._load_existing_sd_files()
            else:
                self.current_photo_dir = "A:/DCIM/100MEDIA"
        except Exception as e:
            self.logger.warn(f"SD card init error: {e}")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
    
    def _load_existing_sd_files(self):
        """Mark existing SD files as seen"""
        try:
            files = self.camera.get_media_list(self.current_photo_dir)
            with self.download_lock:
                for file_info in files:
                    filename = file_info.get('name', '')
                    if filename:
                        self.downloaded_files.add(filename)
        except Exception as e:
            self.logger.warn(f"Could not load existing files: {e}")
    
    def execute_pipeline(self, filename_override: Optional[str] = None) -> bool:
        """Convenience wrapper: run phases 1+2+3 in sequence.""" 
        file_info = self.capture_and_index()
        if file_info is None:
            return False
        result = self.download_and_save(
            file_info, filename_override=filename_override
        )
        return result is not None

    def capture_and_index(self) -> Optional[Dict]:
        """Phases 1+2: fire shutter, poll SD card for the new file."""
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
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING

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
        with self.state_lock:
            self.pipeline_state = CaptureState.DOWNLOADING

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
        """Phase 1: Trigger camera capture"""
        try:
            self.camera.send_capture_command(self.current_resolution)
            with self.state_lock:
                self.photo_count += 1
            return True
        except CameraConnectionError as e:
            self.logger.error(f"[Phase 1] Failed: {e}")
            return False
    
    def _phase2_index(self, timeout: float = CAPTURE_TIMEOUT_SECONDS) -> Optional[Dict]:
        """Phase 2: Poll SD card for new image (simplified - no backoff)"""
        start_time = time.time()
        
        while (time.time() - start_time) < timeout: # 15s
            if new_file := self.camera.get_new_file(): # := is the walrus op. means assign and check at the same time.
                self.logger.info(f"Found new image: {new_file.get('name')}")
                return new_file
            time.sleep(SD_POLL_INTERVAL)
        
        self.logger.error(f"Timeout after {timeout}s")
        return None
    
    def _find_new_file(self) -> Optional[Dict]:
        """Find first unclaimed file and claim it atomically."""
        try:
            file_list = self.camera.get_media_list(self.current_photo_dir)

            with self.download_lock:
                for file_info in reversed(file_list):
                    filename = file_info.get('name', '')
                    if filename and filename not in self.downloaded_files:
                        # Claim now so concurrent walkers skip this file
                        # while phase 3 is still downloading it.
                        self.downloaded_files.add(filename)
                        self.last_photo_count = len(file_list)
                        return file_info
        except Exception:
            pass
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
        img = cv2.rotate(img, cv2.ROTATE_180)
        
        if not self.storage.verify_image_dimensions(img, self.current_resolution):
            self.logger.warn(f"Image dimensions below expected for {self.current_resolution}")
        
        return img if self.storage.verify_image_integrity(img) else None
    
    def get_stats(self) -> Dict:
        with self.state_lock: # lock state_lock, copy into state, unlock
            state =  self.pipeline_state
            
        return {
            'state': state,
            'photo_count': self.photo_count,
            'downloaded_files': self.camera.get_downloaded_count(),
            'current_directory': self.camera.current_photo_dir,
            'resolution': self.current_resolution,
        }
