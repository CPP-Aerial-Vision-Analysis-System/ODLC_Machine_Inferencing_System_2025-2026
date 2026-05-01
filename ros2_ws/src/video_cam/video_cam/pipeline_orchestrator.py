#!/usr/bin/env python3
"""SIYI pipeline orchestrator — capture, index, download."""

import time
import numpy as np
import cv2
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
    """Raised when pipeline execution fails"""
    pass

class PipelineOrchestrator:
    """Orchestrates the image capture pipeline"""
    
    def __init__(self, camera: CameraInterface, storage: StorageManager, logger=None):
        self.camera = camera
        self.storage = storage
        self.logger = logger

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

        # self._load_tracking_state()
        # self._log('info', "Pipeline orchestrator initialized")
    
    def _log(self, level: str, message: str):
        if not self.logger:
            return

        try:
            if level.lower() == 'info':
                self.logger.info(message)
            elif level.lower() == 'warn' or level.lower() == 'warning':
                self.logger.warn(message)
            elif level.lower() == 'error':
                self.logger.error(message)
            elif level.lower() == 'debug':
                self.logger.debug(message)
            else:
                # fallback to info
                self.logger.info(message)
        except ValueError:
            # ROS 2 logger can't change severity between calls
            # fallback: print to console
            print(f"{level.upper()}: {message}")
    
    def get_state(self) -> CaptureState:
        with self.state_lock:
            return self.pipeline_state

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
        self._log('info', f"Resolution set to: {resolution}")
    
    def initialize_sd_card(self):
        """Initialize SD card state from camera"""
        self._log('info', "Initializing SD card...")
        
        try:
            directories = self.camera.get_directories()
            # print (directories)
            if directories:
                self.current_photo_dir = directories[-1]['path']
                # self._log('info', f"Photo directory: {self.current_photo_dir}")
                
                if self.last_photo_count == 0:
                    count = self.camera.get_media_count(self.current_photo_dir)
                    if count is not None:
                        self.last_photo_count = count
                    self._load_existing_sd_files()
            else:
                self.current_photo_dir = "A:/DCIM/100MEDIA"
        except Exception as e:
            self._log('warn', f"SD card init error: {e}")
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
            # self._log('info', f"Marked {len(files)} existing files as seen")
        except Exception as e:
            self._log('warn', f"Could not load existing files: {e}")
    
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
            self._log('warn',
                      "capture_and_index: another capture is mid-shutter, dropping trigger")
            return None

        start_time = time.time()
        try:
            # self._log('info', "STARTING CAPTURE (phases 1+2)")

            # Phase 1: trigger camera
            if not self._phase1_capture():
                self._log('error', "Phase 1 failed: capture command rejected")
                return None

            time.sleep(0.5)  # Brief wait for SD write

            # Phase 2: index SD card
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING

            file_info = self._phase2_index()
            if not file_info:
                self._log('error', "Phase 2 failed: new image not found on SD card")
                return None

            elapsed = time.time() - start_time
            # self._log('info', f"Phases 1+2 complete in {elapsed:.1f}s")
            return file_info
        except Exception as e:
            self._log('error', f"capture_and_index exception: {e}")
            return None
        finally:
            self.capture_lock.release()

    def download_and_save(
        self,
        file_info: Dict,
        filename_override: Optional[str] = None,
    ) -> Optional[Tuple[str, np.ndarray]]:
        """Phase 3: download bytes, decode, save atomically."""
        start_time = time.time()
        with self.state_lock:
            self.pipeline_state = CaptureState.DOWNLOADING

        try:
            saved_path, img = self._phase3_download(
                file_info, filename_override=filename_override
            )
        except Exception as e:
            self._log('error', f"download_and_save exception: {e}")
            return None

        if not saved_path or img is None:
            self._log('error', "Phase 3 failed: download or save error")
            return None

        self.last_saved_path = saved_path
        elapsed = time.time() - start_time
        # self._log('info', f"Phase 3 complete in {elapsed:.1f}s: {saved_path}")
        return saved_path, img
    
    def _phase1_capture(self) -> bool:
        """Phase 1: Trigger camera capture"""
        try:
            self.camera.send_capture_command(self.current_resolution)
            with self.state_lock:
                self.photo_count += 1
            # self._log('info', f"[Phase 1] Capture command sent (photo #{self.photo_count})")
            return True
        except CameraConnectionError as e:
            self._log('error', f"[Phase 1] Failed: {e}")
            return False
    
    def _phase2_index(self, timeout: float = CAPTURE_TIMEOUT_SECONDS) -> Optional[Dict]:
        """Phase 2: Poll SD card for new image (simplified - no backoff)"""
        # self._log('info', f"[Phase 2] Polling SD card...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if new_file := self._find_new_file():
                self._log('info', f"Found new image: {new_file.get('name')}")
                return new_file
            time.sleep(SD_POLL_INTERVAL)
        
        self._log('error', f"Timeout after {timeout}s")
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
    
    def _phase3_download(
        self,
        file_info: Dict,
        filename_override: Optional[str] = None,
    ) -> tuple:
        """Phase 3: Download and save."""
        original_name = file_info.get('name', '')
        file_url = file_info.get('url', '')

        if not original_name or not file_url:
            return None, None

        save_name = filename_override or original_name

        # if save_name != original_name:
        #     self._log('info',
        #               f"[Phase 3] Downloading {original_name} (saving as {save_name})")
        # else:
        #     self._log('info', f"[Phase 3] Downloading: {original_name}")

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
            self._log('error', f"Insufficient disk space (need {required_mb:.1f}MB)")
            return None
        
        return self.camera.download_image(file_info.get('url', ''))
    
    def _decode_and_verify(self, image_bytes: bytes) -> Optional[np.ndarray]:
        """Decode, verify, and rotate image (camera is mounted upside down)"""
        img = self.camera.decode_image(image_bytes)
        if img is None:
            return None
        # I did this because the camera is upside down all the time (might not need this if its gonna work properly during flight)    
        img = cv2.rotate(img, cv2.ROTATE_180)
        
        if not self.storage.verify_image_dimensions(img, self.current_resolution):
            self._log('warn', f"Image dimensions below expected for {self.current_resolution}")
        
        return img if self.storage.verify_image_integrity(img) else None
    
    def get_stats(self) -> Dict:
        """Get pipeline statistics"""
        with self.download_lock:
            num_downloaded = len(self.downloaded_files)
        
        return {
            'state': self.get_state(),
            'photo_count': self.photo_count,
            'downloaded_files': num_downloaded,
            'current_directory': self.current_photo_dir,
            'resolution': self.current_resolution,
        }