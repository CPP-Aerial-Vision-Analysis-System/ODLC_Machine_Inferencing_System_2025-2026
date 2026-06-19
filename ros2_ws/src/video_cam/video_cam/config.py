#!/usr/bin/env python3
"""SIYI A8 Mini configuration constants."""

from enum import Enum
from typing import Dict


class MediaTypes(Enum):
    IMAGE = 0
    VIDEO = 1


# Hardware
CAMERA_IP = "192.168.144.25"
CONTROL_PORT = 37260 # UDP port
MEDIA_PORT = 82
PHOTO_RESOLUTIONS = {'4K': 0x00, '2.7K': 0x01, '1080P': 0x02}
VERIFIED_RESOLUTIONS = {'4K'}

CAPTURE_COMMANDS = {
    '4K': bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce"),
    '2.7K': bytes.fromhex("55 66 01 01 00 00 00 0c 01 35 ce"),
    '1080P': bytes.fromhex("55 66 01 01 00 00 00 0c 02 36 ce"),
}

# SIYI SDK protocol constants
SDK_STX = b'\x55\x66'

# SIYI SDK command IDs i found from the documentation
CMD_AUTO_FOCUS = 0x04
CMD_MANUAL_ZOOM_AF = 0x05
CMD_MANUAL_FOCUS = 0x06
CMD_GIMBAL_ROTATE = 0x07
CMD_GIMBAL_CENTER = 0x08
CMD_FUNCTION_FEEDBACK = 0x0B
CMD_CAPTURE_RECORD = 0x0C
CMD_GIMBAL_ATTITUDE = 0x0D
CMD_SET_GIMBAL_ANGLES = 0x0E
CMD_ABSOLUTE_ZOOM_AF = 0x0F
CMD_SUPPORTED_ZOOM_RANGE = 0x16
CMD_CURRENT_ZOOM = 0x18
CMD_GIMBAL_MODE = 0x19
CMD_STREAM_CONFIG = 0x25
CMD_SINGLE_AXIS_CONTROL = 0x41
CMD_FORMAT_SD_CARD = 0x48

# 0x0C function types for gimbal mode
FUNC_TYPE_SET_LOCK_MODE = 3
FUNC_TYPE_SET_FOLLOW_MODE = 4
FUNC_TYPE_SET_FPV_MODE = 5

GIMBAL_MODE_TO_FUNC_TYPE = {
    'lock': FUNC_TYPE_SET_LOCK_MODE,
    'follow': FUNC_TYPE_SET_FOLLOW_MODE,
    'fpv': FUNC_TYPE_SET_FPV_MODE,
}

GIMBAL_MODE_LABELS = {
    0: 'lock',
    1: 'follow',
    2: 'fpv',
}

STREAM_TYPE_LASER = 2

# Image specs
RESOLUTION_SPECS: Dict[str, Dict[str, int]] = {
    '4K': {'min_width': 3000, 'min_height': 1600, 'min_file_size': 50000},
    '2.7K': {'min_width': 2000, 'min_height': 1200, 'min_file_size': 30000},
    '1080P': {'min_width': 1800, 'min_height': 900, 'min_file_size': 20000}
}

# Timing
CAPTURE_TIMEOUT_SECONDS = 15.0
SD_POLL_INTERVAL = 0.5
HTTP_TIMEOUT_SECONDS = 10.0
SDK_SOCKET_TIMEOUT_SECONDS = 2.0
NODE_LOOP_PERIOD = 1.0  # Main loop period (1 Hz for capture polling)
HEALTH_CHECK_PERIOD = 5.0  # Camera health + disk status check interval

# Storage
MIN_FREE_SPACE_MB = 50
REQUIRED_DOWNLOAD_SPACE_MB = 10
MAPPING_SUBDIR = "mapping_photos"  # Single directory for all images
MIN_FILE_SIZE_BYTES = 1000
ATOMIC_WRITE_SUFFIX = ".tmp"

# HTTP
HTTP_POOL_CONNECTIONS = 1
HTTP_POOL_MAXSIZE = 3
HTTP_MAX_RETRIES = 0
HTTP_HEADERS = {'User-Agent': 'SIYI-ROS-Client/1.0', 'Connection': 'keep-alive'}

# ROS defaults
DEFAULT_USE_REAL_CAMERA = True
DEFAULT_MIN_ALTITUDE_AGL = -13.716
DEFAULT_RESOLUTION = '4K'
DEFAULT_ROTATE_180 = True
