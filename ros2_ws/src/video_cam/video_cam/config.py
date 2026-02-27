#!/usr/bin/env python3
"""SIYI A8 Mini configuration constants."""

from enum import Enum
from typing import Dict


class MediaTypes(Enum):
    IMAGE = 0
    VIDEO = 1


class CaptureState(Enum):
    IDLE = "idle"
    CAPTURING = "capturing"
    INDEXING = "indexing"
    DOWNLOADING = "downloading"
    FAILED = "failed"


# Hardware
CAMERA_IP = "192.168.144.25"
CONTROL_PORT = 37260
MEDIA_PORT = 82
PHOTO_RESOLUTIONS = {'4K': 0x00, '2.7K': 0x01, '1080P': 0x02}
VERIFIED_RESOLUTIONS = {'4K'}

CAPTURE_COMMANDS = {
    '4K': bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce"),
    '2.7K': bytes.fromhex("55 66 01 01 00 00 00 0c 01 35 ce"),
    '1080P': bytes.fromhex("55 66 01 01 00 00 00 0c 02 36 ce"),
}

# Image specs
RESOLUTION_SPECS: Dict[str, Dict[str, int]] = {
    '4K': {'min_width': 3000, 'min_height': 1600, 'min_file_size': 50000},
    '2.7K': {'min_width': 2000, 'min_height': 1200, 'min_file_size': 30000},
    '1080P': {'min_width': 1800, 'min_height': 900, 'min_file_size': 20000}
}

# Timing
CAPTURE_TIMEOUT_SECONDS = 15.0
SD_POLL_INTERVAL = 0.5
MAX_PIPELINE_DURATION = 60.0
HTTP_TIMEOUT_SECONDS = 10.0
SDK_SOCKET_TIMEOUT_SECONDS = 2.0
NODE_LOOP_PERIOD = 1.0  # Main loop period (1 Hz is plenty for capture polling + disk status)

# Storage
MIN_FREE_SPACE_MB = 50
REQUIRED_DOWNLOAD_SPACE_MB = 10
WORKSPACE_SUBDIR = "video_cam"
MAPPING_SUBDIR = "mapping_photos"  # Single directory for all images
TRACKING_STATE_FILE = ".tracking_state.json"
MIN_FILE_SIZE_BYTES = 1000
ATOMIC_WRITE_SUFFIX = ".tmp"

# JPEG validation constants
JPEG_HEADER_BYTES = b'\xff\xd8\xff'  # JPEG file signature (SOI + start of frame)
JPEG_FOOTER_BYTES = b'\xff\xd9'      # JPEG end of image marker (EOI)

# HTTP
HTTP_POOL_CONNECTIONS = 1
HTTP_POOL_MAXSIZE = 3
HTTP_MAX_RETRIES = 0
HTTP_HEADERS = {'User-Agent': 'SIYI-ROS-Client/1.0', 'Connection': 'keep-alive'}

# ROS defaults
DEFAULT_USE_REAL_CAMERA = True
DEFAULT_MIN_ALTITUDE_AGL = -13.716
DEFAULT_CAMERA_IP = CAMERA_IP
DEFAULT_CTRL_PORT = CONTROL_PORT
DEFAULT_MEDIA_PORT = MEDIA_PORT
DEFAULT_HTTP_TIMEOUT = HTTP_TIMEOUT_SECONDS
DEFAULT_CAPTURE_TIMEOUT = CAPTURE_TIMEOUT_SECONDS
DEFAULT_MIN_FREE_SPACE_MB = MIN_FREE_SPACE_MB

