#!/usr/bin/env python3
"""SIYI Camera interface —> SDK + HTTP communication."""

import binascii #ask
import cv2
import socket
import struct #ask
import requests
import numpy as np
import rclpy.logging
from threading import Lock
from typing import Optional, List, Dict, Set, Any
from .config import (
    CAMERA_IP,
    CONTROL_PORT,
    MEDIA_PORT,
    MediaTypes,
    CAPTURE_COMMANDS,
    TAKE_PHOTO_COMMAND,
    DEFAULT_RESOLUTION,
    CMD_HARDWARE_ID,
    HARDWARE_ID_TO_MODEL,
    HTTP_TIMEOUT_SECONDS,
    SDK_SOCKET_TIMEOUT_SECONDS,
    HTTP_POOL_CONNECTIONS,
    HTTP_POOL_MAXSIZE,
    HTTP_MAX_RETRIES,
    HTTP_HEADERS,
    SDK_STX,
    CMD_AUTO_FOCUS,
    CMD_MANUAL_ZOOM_AF,
    CMD_MANUAL_FOCUS,
    CMD_GIMBAL_ROTATE,
    CMD_GIMBAL_CENTER,
    CMD_FUNCTION_FEEDBACK,
    CMD_CAPTURE_RECORD,
    CMD_SYSTEM_INFO,
    RECORD_STA_NO_TF_CARD,
    RECORD_STA_LABELS,
    FUNCTION_FEEDBACK_LABELS,
    CMD_GIMBAL_ATTITUDE,
    CMD_SET_GIMBAL_ANGLES,
    CMD_ABSOLUTE_ZOOM_AF,
    CMD_SUPPORTED_ZOOM_RANGE,
    CMD_CURRENT_ZOOM,
    CMD_GIMBAL_MODE,
    CMD_STREAM_CONFIG,
    CMD_SINGLE_AXIS_CONTROL,
    CMD_FORMAT_SD_CARD,
    GIMBAL_MODE_TO_FUNC_TYPE,
    GIMBAL_MODE_LABELS,
    STREAM_TYPE_LASER,
)


    # Raised when camera connection fails
class CameraConnectionError(Exception):
    pass


class CameraInterface:
    """Low-level interface to SIYI A8 Mini camera."""

    def __init__(self, camera_ip: str = CAMERA_IP,
                 ctrl_port: int = CONTROL_PORT,
                 media_port: int = MEDIA_PORT,
                 http_timeout: float = HTTP_TIMEOUT_SECONDS,
                 logger=None):
        self.camera_ip = camera_ip
        self.ctrl_port = ctrl_port
        self.media_port = media_port
        self.http_timeout = http_timeout
        self.logger = logger or rclpy.logging.get_logger('CameraInterface')
        self.sdk_seq = 0
        
        # SD Card Tracking
        self.current_photo_dir: Optional[str] = None
        self.last_photo_count: int = 0
        self.downloaded_files: Set[str] = set()
        self.download_lock = Lock()
        
        # API base URL
        self.base_url = f"http://{self.camera_ip}:{self.media_port}/cgi-bin/media.cgi/api/v1"
        # base url is http://192.168.144.25:82/cgi-bin/media.cgi/api/v1
        
        # SDK socket (UDP)
        # Creates network socket object the sends and receives packets
        # af_inet = ipv4 address family, docker sock_dgram = udp diagram type, sock_stream would be tcp
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

    def _next_seq(self) -> int:
        seq = self.sdk_seq # packet number to track commands and responses
        self.sdk_seq = (self.sdk_seq + 1) & 0xFFFF # increase the sequence number and have a cap of 16 bits
        return seq

    def _compute_crc16(self, payload: bytes) -> int:
        # SIYI SDK uses CRC-16/CCITT (poly 0x1021, init 0x0000).
        return binascii.crc_hqx(payload, 0x0000)

    def _build_sdk_packet(self, cmd_id: int, data: bytes = b'',
                          need_ack: bool = True) -> bytes:
        # 0x01 = camera should reply back, 0x00 = no need to reply
        ctrl = 0x01 if need_ack else 0x00
        data_len = len(data)
        seq = self._next_seq()

        packet_wo_crc = (
            SDK_STX
            + bytes([ctrl])
            + data_len.to_bytes(2, 'little')
            + seq.to_bytes(2, 'little')
            + bytes([cmd_id & 0xFF])
            + data
        )

        crc = self._compute_crc16(packet_wo_crc)
        return packet_wo_crc + crc.to_bytes(2, 'little')

    def _parse_sdk_packet(self, payload: bytes) -> Optional[Dict[str, Any]]:
        if len(payload) < 10 or payload[:2] != SDK_STX:
            return None

        ctrl = payload[2]
        data_len = int.from_bytes(payload[3:5], 'little')
        seq = int.from_bytes(payload[5:7], 'little')
        cmd_id = payload[7]

        frame_len = 8 + data_len + 2
        if len(payload) < frame_len:
            return None

        data = payload[8:8 + data_len]
        crc_recv = int.from_bytes(payload[8 + data_len:10 + data_len], 'little')
        crc_calc = self._compute_crc16(payload[:8 + data_len])

        if crc_recv != crc_calc:
            self.logger.warn(
                      f"SDK CRC mismatch: recv={crc_recv:04x}, calc={crc_calc:04x}")
            return None

        return {
            'ctrl': ctrl,
            'seq': seq,
            'cmd_id': cmd_id,
            'data': data,
        }

    def _receive_sdk_packet(self,
                            expected_cmd_ids: Optional[Set[int]] = None,
                            timeout: Optional[float] = None,
                            max_frames: int = 8) -> Optional[Dict[str, Any]]:
        # waits for a UDP reply from the camera
        original_timeout = self.sdk_socket.gettimeout()
        if timeout is None:
            timeout = original_timeout

        self.sdk_socket.settimeout(timeout)
        try:
            for _ in range(max_frames):
                response, _ = self.sdk_socket.recvfrom(2048)
                parsed = self._parse_sdk_packet(response)
                if not parsed:
                    continue
                if expected_cmd_ids and parsed['cmd_id'] not in expected_cmd_ids:
                    continue
                return parsed 
            return None
        except socket.timeout:
            return None
        finally:
            self.sdk_socket.settimeout(original_timeout)

    def _send_sdk_command(self, cmd_id: int, data: bytes = b'',
                          expect_ack: bool = True,
                          expected_cmd_ids: Optional[Set[int]] = None,
                          timeout: Optional[float] = None
                          ) -> Optional[Dict[str, Any]]:
        packet = self._build_sdk_packet(cmd_id, data=data, need_ack=expect_ack)
        self.sdk_socket.sendto(packet, (self.camera_ip, self.ctrl_port))

        if not expect_ack:
            return None

        if expected_cmd_ids is None:
            expected_cmd_ids = {cmd_id}

        return self._receive_sdk_packet(expected_cmd_ids, timeout=timeout)

    def _status_ok(self, response: Optional[Dict[str, Any]],
                   success_value: int = 1) -> bool:
        if not response:
            return False

        data = response.get('data', b'')
        if not data:
            return True

        return data[0] == success_value
        
    def send_capture_command(self, resolution: str = DEFAULT_RESOLUTION) -> bool:
        # Send capture command to camera via UDP SDK.
        try:
            # `resolution` is only a validation profile - the still size is fixed
            # by the sensor and no SDK command changes it, so every capture is
            # the same 0x0C func_type=0 packet. Never send func_type 1 or 2 here:
            # those toggle HDR and start/stop video recording.
            capture_command = CAPTURE_COMMANDS.get(resolution, TAKE_PHOTO_COMMAND)

            # Send UDP packet for the camera (actual signal to capture)
            self.sdk_socket.sendto(capture_command, (self.camera_ip, self.ctrl_port))
            
            # Capture command often reports via 0x0B function feedback.
            feedback = self._receive_sdk_packet(
                expected_cmd_ids={CMD_CAPTURE_RECORD, CMD_FUNCTION_FEEDBACK},
                timeout=0.8,
            )

            if not feedback:
                self.logger.warn( "No capture feedback received (timeout), assuming success")
                return True

            if feedback['cmd_id'] == CMD_FUNCTION_FEEDBACK and feedback['data']:
                info_type = feedback['data'][0]
                if info_type in (1, 4):
                    reason = FUNCTION_FEEDBACK_LABELS.get(
                        info_type, f'unknown feedback {info_type}')
                    self.logger.error(f"Capture rejected by camera: {reason}")
                    if not self.sd_card_present():
                        self.logger.error(
                            "Camera reports NO TF CARD. Power the camera down, "
                            "reseat the microSD card, and format it in the "
                            "camera/SIYI app (FAT32 or exFAT). The HTTP media "
                            "API can still list old directories with the card "
                            "unmounted, so a directory listing is not proof "
                            "the card is usable.")
                    return False

            return True
                
        except Exception as e:
            error_msg = f"Capture command failed: {e}"
            self.logger.error( error_msg)
            raise CameraConnectionError(error_msg)

    def auto_focus(self, touch_x: int = 0, touch_y: int = 0) -> bool:
        """Trigger one-time autofocus. Optional touch point can be provided."""
        touch_x = max(0, min(65535, int(touch_x)))
        touch_y = max(0, min(65535, int(touch_y)))
        payload = struct.pack('<BHH', 1, touch_x, touch_y)
        response = self._send_sdk_command(CMD_AUTO_FOCUS, payload)
        return self._status_ok(response)

    def manual_zoom(self, direction: int) -> Optional[float]:
        """Manual zoom control (-1 out, 0 stop, 1 in). Returns current zoom."""
        if direction not in (-1, 0, 1):
            raise ValueError("direction must be one of -1, 0, 1")

        payload = struct.pack('<b', direction)
        response = self._send_sdk_command(CMD_MANUAL_ZOOM_AF, payload)
        if not response or len(response['data']) < 2:
            return None

        zoom_raw = struct.unpack('<H', response['data'][:2])[0]
        return zoom_raw / 10.0

    def manual_focus(self, direction: int) -> bool:
        """Manual focus control (-1 near, 0 stop, 1 far)."""
        if direction not in (-1, 0, 1):
            raise ValueError("direction must be one of -1, 0, 1")

        payload = struct.pack('<b', direction)
        response = self._send_sdk_command(CMD_MANUAL_FOCUS, payload)
        return self._status_ok(response)

    def absolute_zoom_autofocus(self, zoom_multiple: float) -> bool:
        """Set absolute zoom (e.g. 4.5x), with autofocus on supported models."""
        if zoom_multiple <= 0:
            raise ValueError("zoom_multiple must be > 0")

        zoom_int = int(zoom_multiple)
        zoom_float = int(round((zoom_multiple - zoom_int) * 10))
        if zoom_float == 10:
            zoom_int += 1
            zoom_float = 0

        zoom_int = max(0, min(255, zoom_int))
        zoom_float = max(0, min(9, zoom_float))

        payload = struct.pack('<BB', zoom_int, zoom_float)
        response = self._send_sdk_command(CMD_ABSOLUTE_ZOOM_AF, payload)
        return self._status_ok(response)

    def get_hardware_id(self) -> Optional[str]:
        """Return the raw hardware-ID string reported by the gimbal (CMD 0x02)."""
        response = self._send_sdk_command(CMD_HARDWARE_ID)
        if not response or not response.get('data'):
            return None

        try:
            return response['data'].decode('ascii', errors='ignore').strip('\x00').strip()
        except Exception:
            return None

    def detect_model(self) -> Optional[str]:
        """Ask the camera which model it is.

        The first two characters of the hardware ID are the model number in
        hex (e.g. '6B...' -> ZR10, '73...' -> A8 mini). Returns a key of
        CAMERA_MODELS, or None if the camera did not answer or is unknown.
        """
        try:
            hw_id = self.get_hardware_id()
        except Exception as e:
            self.logger.warn(f"Hardware ID query failed: {e}")
            return None

        if not hw_id or len(hw_id) < 2:
            self.logger.warn("No hardware ID returned by camera")
            return None

        try:
            model_code = int(hw_id[:2], 16)
        except ValueError:
            self.logger.warn(f"Unparsable hardware ID '{hw_id}'")
            return None

        model = HARDWARE_ID_TO_MODEL.get(model_code)
        if model is None:
            self.logger.warn(
                f"Unknown camera model code 0x{model_code:02X} (hw_id='{hw_id}')")
            return None

        self.logger.info(f"Detected camera: {model} (hw_id='{hw_id}')")
        return model

    def get_supported_zoom_range(self) -> Optional[Dict[str, float]]:
        """Return current max supported zoom as {'max_zoom': value}."""
        response = self._send_sdk_command(CMD_SUPPORTED_ZOOM_RANGE)
        if not response or len(response['data']) < 2:
            return None

        data = response['data']
        max_zoom = data[0] + (data[1] / 10.0)
        return {'max_zoom': max_zoom}

    def get_current_zoom_magnification(self) -> Optional[float]:
        """Return current zoom magnification (e.g. 4.5)."""
        response = self._send_sdk_command(CMD_CURRENT_ZOOM)
        if not response or len(response['data']) < 2:
            return None

        data = response['data']
        return data[0] + (data[1] / 10.0)

    def rotate_gimbal(self, yaw_speed: int = 0, pitch_speed: int = 0) -> bool:
        """Rotate gimbal using speed command in range -100 to 100."""
        yaw_speed = max(-100, min(100, int(yaw_speed)))
        pitch_speed = max(-100, min(100, int(pitch_speed)))
        payload = struct.pack('<bb', yaw_speed, pitch_speed)
        response = self._send_sdk_command(CMD_GIMBAL_ROTATE, payload)
        return self._status_ok(response)

    def stop_gimbal_rotation(self) -> bool:
        """Stop yaw/pitch rotation by sending zero speed."""
        return self.rotate_gimbal(yaw_speed=0, pitch_speed=0)

    def center_gimbal(self) -> bool:
        """Center gimbal to its zero position."""
        response = self._send_sdk_command(CMD_GIMBAL_CENTER, bytes([1]))
        return self._status_ok(response)

    def request_gimbal_attitude(self) -> Optional[Dict[str, float]]:
        """Get yaw/pitch/roll angles and angular velocities."""
        response = self._send_sdk_command(CMD_GIMBAL_ATTITUDE)
        if not response or len(response['data']) < 12:
            return None

        values = struct.unpack('<hhhhhh', response['data'][:12])
        return {
            'yaw_deg': values[0] / 10.0,
            'pitch_deg': values[1] / 10.0,
            'roll_deg': values[2] / 10.0,
            'yaw_vel_dps': values[3] / 10.0,
            'pitch_vel_dps': values[4] / 10.0,
            'roll_vel_dps': values[5] / 10.0,
        }

    def set_gimbal_angles(self, yaw_deg: float, pitch_deg: float
                          ) -> Optional[Dict[str, float]]:
        """Set target yaw/pitch angles. Roll is returned but not directly set."""
        yaw_raw = int(round(float(yaw_deg) * 10.0))
        pitch_raw = int(round(float(pitch_deg) * 10.0))
        payload = struct.pack('<hh', yaw_raw, pitch_raw)

        response = self._send_sdk_command(CMD_SET_GIMBAL_ANGLES, payload)
        if not response or len(response['data']) < 6:
            return None

        current = struct.unpack('<hhh', response['data'][:6])
        return {
            'yaw_deg': current[0] / 10.0,
            'pitch_deg': current[1] / 10.0,
            'roll_deg': current[2] / 10.0,
        }

    def set_single_axis_angle(self, axis: str, angle_deg: float
                              ) -> Optional[Dict[str, float]]:
        """Set a single axis angle (yaw or pitch) using command 0x41."""
        # Per SDK appendix, A8 mini acknowledges 0x41 with cmd_id 0x0E,
        # so we accept either cmd_id as a valid ACK.
        axis_norm = axis.strip().lower()
        if axis_norm == 'yaw':
            axis_flag = 0
        elif axis_norm == 'pitch':
            axis_flag = 1
        else:
            raise ValueError("axis must be 'yaw' or 'pitch'")

        angle_raw = int(round(float(angle_deg) * 10.0))
        payload = struct.pack('<hB', angle_raw, axis_flag)

        response = self._send_sdk_command(
            CMD_SINGLE_AXIS_CONTROL,
            payload,
            expected_cmd_ids={CMD_SINGLE_AXIS_CONTROL, CMD_SET_GIMBAL_ANGLES},
        )
        if not response or len(response['data']) < 6:
            return None

        current = struct.unpack('<hhh', response['data'][:6])
        return {
            'yaw_deg': current[0] / 10.0,
            'pitch_deg': current[1] / 10.0,
            'roll_deg': current[2] / 10.0,
        }

    def get_system_info(self) -> Optional[Dict[str, Any]]:
        """Query 0x0A camera system info (HDR, record/TF-card state, mode)."""
        response = self._send_sdk_command(CMD_SYSTEM_INFO)
        if not response or len(response['data']) < 4:
            return None

        data = response['data']
        record_sta = data[3]
        return {
            'hdr_on': bool(data[1]),
            'record_sta': record_sta,
            'record_sta_label': RECORD_STA_LABELS.get(
                record_sta, f'unknown({record_sta})'),
            'sd_card_present': record_sta != RECORD_STA_NO_TF_CARD,
            'gimbal_motion_mode': GIMBAL_MODE_LABELS.get(data[4]) if len(data) > 4 else None,
        }

    def sd_card_present(self) -> Optional[bool]:
        """True/False if the camera can see its TF card, None if unreachable."""
        info = self.get_system_info()
        return None if info is None else info['sd_card_present']

    def get_gimbal_mode(self) -> Optional[str]:
        """Query current gimbal motion mode (lock/follow/fpv)."""
        response = self._send_sdk_command(CMD_GIMBAL_MODE)
        if not response or not response['data']:
            return None

        mode_val = response['data'][0]
        return GIMBAL_MODE_LABELS.get(mode_val, f'unknown({mode_val})')

    def set_gimbal_motion_mode(self, mode: str) -> bool:
        """Set gimbal mode via 0x0C function type (lock, follow, fpv)."""
        mode_key = mode.strip().lower()
        if mode_key not in GIMBAL_MODE_TO_FUNC_TYPE:
            raise ValueError("mode must be one of: lock, follow, fpv")

        func_type = GIMBAL_MODE_TO_FUNC_TYPE[mode_key]

        # 0x0C has no ACK in protocol docs, optional 0x0B feedback may follow.
        self._send_sdk_command(
            CMD_CAPTURE_RECORD,
            data=bytes([func_type]),
            expect_ack=False,
        )

        feedback = self._receive_sdk_packet(
            expected_cmd_ids={CMD_FUNCTION_FEEDBACK},
            timeout=0.4,
        )
        if feedback and feedback['data']:
            info_type = feedback['data'][0]
            if info_type in (1, 4):
                return False

        return True

    def format_sd_card(self) -> bool:
        """Format camera SD card (destructive operation)."""
        # A8 mini (and ZT30/ZR30) do not respond to 0x48 per SDK appendix.
        self._send_sdk_command(CMD_FORMAT_SD_CARD, expect_ack=False)
        return True
    
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
            self.logger.error( f"Cannot connect to camera HTTP API: {e}")
            return []
        except Exception as e:
            self.logger.warn( f"Directory query error: {e}")
            return []
    
    def get_media_list(self, dir_path: str, media_type: MediaTypes = MediaTypes.IMAGE, start: int = 0, count: int = 9999) -> List[Dict]:
        # Sends HTTP req saying give me the media files inside this directory.
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
            self.logger.warn( f"Media list query error: {e}")
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
            self.logger.warn( f"Photo count query error: {e}")
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
                self.logger.error( 
                    f"Download failed: HTTP {response.status_code}: {response.reason}")
                return None
                
        except Exception as e:
            self.logger.error( f"Download error: {e}")
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
            self.logger.error( f"Image decode error: {e}")
            return None
            
    def initialize_sd_card(self):
        """Initialize SD card state from camera"""
        self.logger.info( "Initializing SD card...")

        # Ask the camera itself before trusting the HTTP media API: the API
        # happily lists directories for a card the firmware has not mounted,
        # which is exactly the state where every capture fails.
        info = self.get_system_info()
        if info is None:
            self.logger.warn(
                "Camera did not answer the 0x0A system-info query - cannot "
                "confirm the TF card is mounted.")
        elif not info['sd_card_present']:
            self.logger.error(
                "Camera reports NO TF CARD. Reseat and formatted in the "
                "camera (FAT32/exFAT).")

        try:
            directories = self.get_directories()
            if directories:
                self.current_photo_dir = directories[-1]['path']
                self.logger.info( f"Photo directory: {self.current_photo_dir}")
                
                if self.last_photo_count == 0:
                    count = self.get_media_count(self.current_photo_dir)
                    if count is not None:
                        self.last_photo_count = count
                    self._load_existing_sd_files()
            else:
                self.current_photo_dir = "A:/DCIM/100MEDIA"
        except Exception as e:
            self.logger.warn( f"SD card init error: {e}")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
    
    def _load_existing_sd_files(self):
        """Mark existing SD files as seen"""
        try:
            files = self.get_media_list(self.current_photo_dir)
            with self.download_lock:
                for file_info in files:
                    filename = file_info.get('name', '')
                    if filename:
                        self.downloaded_files.add(filename)
            self.logger.info( f"Marked {len(files)} existing files as seen")
        except Exception as e:
            self.logger.warn( f"Could not load existing files: {e}")

    def get_new_file(self) -> Optional[Dict]:
        # Find first undownloaded file
        if not self.current_photo_dir:
            return None
            
        try:
            file_list = self.get_media_list(self.current_photo_dir)
            current_count = len(file_list)
            
            if current_count > self.last_photo_count:
                with self.download_lock: # download all the files that has not been yet downloaded
                    for file_info in reversed(file_list):
                        filename = file_info.get('name', '')
                        if filename and filename not in self.downloaded_files:
                            self.last_photo_count = current_count
                            return file_info
        except Exception:
            pass
        return None
        
    def mark_file_downloaded(self, filename: str):
        """Mark a file as downloaded"""
        with self.download_lock:
            self.downloaded_files.add(filename)
            
    def get_downloaded_count(self) -> int:
        with self.download_lock:
            return len(self.downloaded_files)
    
    def ping(self) -> bool:
        """Quick connectivity check via SDK gimbal attitude request."""
        try:
            response = self._send_sdk_command(CMD_GIMBAL_ATTITUDE, timeout=1.0)
            return response is not None
        except Exception:
            return False

    def close(self):
        """Close all connections and release resources."""
        if self.http_session:
            self.http_session.close()
        if self.sdk_socket:
            self.sdk_socket.close()
        self.logger.info( "Camera interface closed")
