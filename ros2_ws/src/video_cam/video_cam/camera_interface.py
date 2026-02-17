#!/usr/bin/env python3
'''Waiter'''
"""SIYI A8 Mini Camera Interface - SDK/HTTP/RTSP communication"""

import cv2
import socket
import requests
import numpy as np
from typing import Optional, List, Dict
from .config import (
    CAMERA_IP,
    CONTROL_PORT,
    MEDIA_PORT,
    RTSP_PORT,
    MediaTypes,
    CAPTURE_COMMANDS,
    VERIFIED_RESOLUTIONS,
    HTTP_TIMEOUT_SECONDS,
    SDK_SOCKET_TIMEOUT_SECONDS,
    HTTP_POOL_CONNECTIONS,
    HTTP_POOL_MAXSIZE,
    HTTP_MAX_RETRIES,
    HTTP_HEADERS,
)


    # Raised when camera connection fails
class CameraConnectionError(Exception):
    pass


class CameraInterface:
    #Low-level interface to SIYI A8 Mini camera
    
    def __init__(self, camera_ip: str = CAMERA_IP, 
                 ctrl_port: int = CONTROL_PORT,
                 media_port: int = MEDIA_PORT,
                 rtsp_port: int = RTSP_PORT,
                 http_timeout: float = HTTP_TIMEOUT_SECONDS,
                 logger=None):
        self.camera_ip = camera_ip
        self.ctrl_port = ctrl_port
        self.media_port = media_port
        self.rtsp_port = rtsp_port
        self.http_timeout = http_timeout
        self.logger = logger
        
        # API base URL
        self.base_url = f"http://{self.camera_ip}:{self.media_port}/cgi-bin/media.cgi/api/v1"
        
        # SDK socket (UDP)
        # Creates network socket object the sends and receives packets
        #af_inet = ipv4 address family, docker sock_dgram = udp diagram type, sock_stream would be tcp
        self.sdk_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk_socket.settimeout(SDK_SOCKET_TIMEOUT_SECONDS) #2s time for recvfrom() to not freez forever
        
        # HTTP session with connection pooling
        self.http_session = requests.Session()
        self.http_session.headers.update(HTTP_HEADERS)
        
        adapter = requests.adapters.HTTPAdapter(
            pool_connections=HTTP_POOL_CONNECTIONS,
            pool_maxsize=HTTP_POOL_MAXSIZE,
            max_retries=HTTP_MAX_RETRIES
        )
        self.http_session.mount('http://', adapter)
        
        # RTSP video capture disabled - not needed for capture/save/detect workflow
        # Only needed for live video preview during flight
        # self.video_capture: Optional[cv2.VideoCapture] = None
        self.video_capture = None  # Disabled
        
    def _log(self, level: str, message: str):
        if self.logger:
            log_func = getattr(self.logger, level, None)
            if log_func:
                log_func(message)
    
    def send_capture_command(self, resolution: str = '4K') -> bool:
        #Send capture command to camera via UDP SDK.
        try:
            if resolution not in VERIFIED_RESOLUTIONS:
                self._log('warn', 
                    f"Resolution {resolution} not verified - using 4K for safety")
                resolution = '4K'
            
            capture_command = CAPTURE_COMMANDS.get(resolution, CAPTURE_COMMANDS['4K'])
            
            # Send UDP packet for the camera (actual signal to capture)
            self.sdk_socket.sendto(capture_command, (self.camera_ip, self.ctrl_port))
            
            # Wait for ACK
            try:
                response, addr = self.sdk_socket.recvfrom(1024)
                
                # Check is there >= 10 bytes from the response (from 3rd and 7th bytes)
                if len(response) >= 10:
                    cmd_id = response[2] if len(response) > 2 else 0
                    status = response[6] if len(response) > 6 else 0
                    
                    if cmd_id == 0x0c and status == 0x00:
                        self._log('info', "Camera ACK: Capture confirmed")
                        return True
                    else:
                        self._log('warn', 
                            f"Camera response: cmd_id={cmd_id:02x}, status={status:02x}")
                        return True  # Assume success
                else:
                    self._log('warn', "Invalid ACK length, assuming success")
                    return True
                    
            except socket.timeout:
                self._log('warn', "No ACK received (timeout), assuming success")
                return True  # Camera might not always send ACK
                
        except Exception as e:
            error_msg = f"Capture command failed: {e}"
            self._log('error', error_msg)
            raise CameraConnectionError(error_msg)
    
    def get_directories(self, media_type: MediaTypes = MediaTypes.IMAGE) -> List[Dict]:
        # Get list of directories on SD card.
        try:
            url = f"{self.base_url}/getdirectories"
            params = {'media_type': media_type.value}
            
            # Send HTTP GET request (just like in web but here its in an internal server of siyi)
            response = self.http_session.get(url, params=params, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    return data.get('data', {}).get('directories', [])
            
            return []
            
        except requests.exceptions.ConnectionError as e:
            self._log('error', f"Cannot connect to camera HTTP API: {e}")
            return []
        except Exception as e:
            self._log('warn', f"Directory query error: {e}")
            return []
    
    def get_media_list(self, dir_path: str, media_type: MediaTypes = MediaTypes.IMAGE, start: int = 0, count: int = 9999) -> List[Dict]:
        # Get list of media files in directory.
        try:
            url = f"{self.base_url}/getmedialist"
            params = {
                'media_type': str(media_type.value),
                'path': dir_path,
                'start': start,
                'count': count
            }
            
            response = self.http_session.get(url, params=params, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    return data.get('data', {}).get('list', [])
            
            return []
            
        except Exception as e:
            self._log('warn', f"Media list query error: {e}")
            return []
    
    def get_media_count(self, dir_path: str,
                       media_type: MediaTypes = MediaTypes.IMAGE) -> Optional[int]:
        # Get total count of media files in directory.
        try:
            url = f"{self.base_url}/getmedialist"
            params = {
                'media_type': str(media_type.value),
                'path': dir_path,
                'start': 0,
                'count': 1
            }
            
            response = self.http_session.get(url, params=params, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    return data.get('data', {}).get('total', 0)
            
            return None
            
        except Exception as e:
            self._log('warn', f"Photo count query error: {e}")
            return None
    
    def download_image(self, file_url: str) -> Optional[bytes]:
        # Download image file from SD card.
        try:
            # Fix IP address in URL if needed
            file_url = file_url.replace("192.168.144.25", self.camera_ip)
            
            response = self.http_session.get(file_url, timeout=self.http_timeout)
            
            if response.status_code == 200:
                return response.content
            else:
                self._log('error', 
                    f"Download failed: HTTP {response.status_code}: {response.reason}")
                return None
                
        except Exception as e:
            self._log('error', f"Download error: {e}")
            return None
    
    def decode_image(self, image_bytes: bytes) -> Optional[np.ndarray]:
        # Decode JPEG bytes to image array.
        # Returns: OpenCV image array (BGR) or None if decode failed
        # this is used to process objrec, publish to ros topics, overaly bounding boxes(od)
        # models require in this format np.ndarray shape: (H, W, 3)
        try:
            img_array = np.frombuffer(image_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            self._log('error', f"Image decode error: {e}")
            return None
    
    def connect_video_stream(self) -> bool:
        """RTSP video stream - DISABLED
        
        Not needed for capture workflow - slows down Jetson unnecessarily.
        We use SDK capture commands, not RTSP stream extraction.
        """
        return False  # Disabled - not needed for capture/save/detect workflow
        
        # Original RTSP connection code (commented out):
        # rtsp_url = f'rtsp://{self.camera_ip}:{self.rtsp_port}/main.264'
        # self._log('info', f"Connecting to camera at {rtsp_url}...")
        # self.video_capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        # if self.video_capture.isOpened():
        #     self.video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        #     try:
        #         timeout_ms = int(self.http_timeout * 1000)
        #         self.video_capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout_ms)
        #         self.video_capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout_ms)
        #         self._log('info', f"Video capture timeouts set to {timeout_ms}ms")
        #     except (AttributeError, Exception) as e:
        #         self._log('warn', f"Video capture timeout not supported: {e}")
        # Fallback to GStreamer (commented out)
        # if not self.video_capture.isOpened():
        #     self._log('warn', "FFmpeg failed, trying GStreamer...")
        #     gst_pipeline = (
        #         f'rtspsrc location={rtsp_url} latency=0 ! '
        #         'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
        #     )
        #     self.video_capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        # Fallback to default backend (commented out)
        # if not self.video_capture.isOpened():
        #     self._log('warn', "GStreamer failed, trying default backend...")
        #     self.video_capture = cv2.VideoCapture(rtsp_url)
        # if self.video_capture.isOpened():
        #     self._log('info', "Camera video stream connected")
        #     return True
        # else:
        #     self._log('error', "Failed to connect to camera video stream")
        #     self.video_capture = None
        #     return False
    
    def read_video_frame(self) -> Optional[np.ndarray]:
        """Read frame from RTSP stream - DISABLED (not needed)"""
        return None  # Disabled - not needed for capture workflow
        
        # Original frame reading code (commented out):
        # if self.video_capture is None or not self.video_capture.isOpened():
        #     return None
        # try:
        #     ret, frame = self.video_capture.read()
        #     if ret and frame is not None:
        #         return frame
        #     return None
        # except Exception as e:
        #     self._log('warn', f"Frame read error: {e}")
        #     return None
    
    def close(self):
        """Close all connections and release resources"""
        if self.http_session:
            self.http_session.close()
        
        if self.sdk_socket:
            self.sdk_socket.close()
        
        # Video capture disabled
        # if self.video_capture:
        #     self.video_capture.release()
        
        self._log('info', "Camera interface closed")
