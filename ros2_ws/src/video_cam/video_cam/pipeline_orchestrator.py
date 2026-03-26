#!/usr/bin/env python3
"""SIYI pipeline orchestrator — capture, index, download."""

import time
import numpy as np
import cv2
from threading import Lock
from contextlib import contextmanager
from typing import Optional, Set, Dict
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
        
        # Simple filename-only tracking
        self.downloaded_files: Set[str] = set()
        self.download_lock = Lock()
        
        self.current_photo_dir: Optional[str] = None
        self.last_photo_count: int = 0
        self.photo_count: int = 0
        self.current_resolution: str = '4K'
        
        # self._load_tracking_state()
        self._log('info', "Pipeline orchestrator initialized")
    
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
    
    @contextmanager
    def _acquire_pipeline(self):
        """Context manager for pipeline state. Always resets to IDLE on exit."""
        with self.state_lock:
            if self.pipeline_state != CaptureState.IDLE:
                raise PipelineError("Pipeline already running")
            self.pipeline_state = CaptureState.CAPTURING

        try:
            yield
        except Exception:
            with self.state_lock:
                self.pipeline_state = CaptureState.FAILED
            time.sleep(1.0)
            raise
        finally:
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
    
    def get_state(self) -> CaptureState:
        with self.state_lock:
            return self.pipeline_state
    
    def is_busy(self) -> bool:
        return self.get_state() != CaptureState.IDLE
    
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
                self._log('info', f"Photo directory: {self.current_photo_dir}")
                
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
            self._log('info', f"Marked {len(files)} existing files as seen")
        except Exception as e:
            self._log('warn', f"Could not load existing files: {e}")
    
    def execute_pipeline(self) -> bool:
        """Execute complete 4-phase capture pipeline"""
        with self._acquire_pipeline():
            return self._run_phases()
    
    def _run_phases(self) -> bool:
        """Run all pipeline phases"""
        start_time = time.time()
        
        try:
            self._log('info', "STARTING CAPTURE PIPELINE")
            
            # Phase 1: Trigger capture
            if not self._phase1_capture():
                raise PipelineError("Capture command failed")
            
            time.sleep(0.5)  # Brief wait for SD write
            
            # Phase 2: Find new image on SD
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING
            
            file_info = self._phase2_index()
            if not file_info:
                raise PipelineError("Image not found on SD card")
            
            # Phase 3: Download and save
            with self.state_lock:
                self.pipeline_state = CaptureState.DOWNLOADING
            
            filename, img = self._phase3_download(file_info)
            if not filename:
                raise PipelineError("Download failed")
            
            # Mark as downloaded
            with self.download_lock:
                self.downloaded_files.add(filename)
            
            elapsed = time.time() - start_time
            self._log('info', f"PIPELINE COMPLETED in {elapsed:.1f}s")
            # self._save_tracking_state()
            return True
            
        except Exception as e:
            elapsed = time.time() - start_time
            self._log('error', f"PIPELINE FAILED after {elapsed:.1f}s: {e}")
            return False
    
    def _phase1_capture(self) -> bool:
        """Phase 1: Trigger camera capture"""
        try:
            self.camera.send_capture_command(self.current_resolution)
            with self.state_lock:
                self.photo_count += 1
            self._log('info', f"[Phase 1] Capture command sent (photo #{self.photo_count})")
            return True
        except CameraConnectionError as e:
            self._log('error', f"[Phase 1] Failed: {e}")
            return False
    
    def _phase2_index(self, timeout: float = CAPTURE_TIMEOUT_SECONDS) -> Optional[Dict]:
        """Phase 2: Poll SD card for new image (simplified - no backoff)"""
        self._log('info', f"[Phase 2] Polling SD card...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            if new_file := self._find_new_file():
                self._log('info', f"Found new image: {new_file.get('name')}")
                return new_file
            time.sleep(SD_POLL_INTERVAL)
        
        self._log('error', f"Timeout after {timeout}s")
        return None
    
    def _find_new_file(self) -> Optional[Dict]:
        """Find first undownloaded file"""
        try:
            file_list = self.camera.get_media_list(self.current_photo_dir)
            current_count = len(file_list)
            
            if current_count > self.last_photo_count:
                with self.download_lock:
                    for file_info in reversed(file_list):
                        filename = file_info.get('name', '')
                        if filename and filename not in self.downloaded_files:
                            self.last_photo_count = current_count
                            return file_info
        except Exception:
            pass
        return None
    
    def _phase3_download(self, file_info: Dict) -> tuple:
        """Phase 3: Download and save (simplified)"""
        filename = file_info.get('name', '')
        file_url = file_info.get('url', '')
        
        if not filename or not file_url:
            return None, None
        
        self._log('info', f"[Phase 3] Downloading: {filename}")
        
        # Download
        image_bytes = self._download_bytes(file_info)
        if not image_bytes:
            return None, None
        
        # Decode and verify
        img = self._decode_and_verify(image_bytes)
        if img is None:
            return None, None
        
        # Save
        if not self.storage.save_image(filename, img, self.current_resolution):
            return None, None
        
        return filename, img
    
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
