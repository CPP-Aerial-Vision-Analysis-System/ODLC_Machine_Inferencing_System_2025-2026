#!/usr/bin/env python3
"""SIYI gimbal camera configuration constants (ZR10 / ZR30 / A8 mini)."""

from enum import Enum
from typing import Dict


class MediaTypes(Enum):
    IMAGE = 0
    VIDEO = 1


# Hardware
CAMERA_IP = "192.168.144.25"
CONTROL_PORT = 37260 # UDP port
MEDIA_PORT = 82
# Still-photo resolution is FIXED BY THE SENSOR. There is no SDK command to
# change it: 0x20 / 0x21 only configure the video stream / recording codecs
# (ZR10 manual v1.7 p.49). So a "resolution" here is only a validation profile
# used to sanity-check what the camera actually wrote to the SD card.
#   ZR10: 1/2.7" 8 MP CMOS, stills 2K 2560x1440   (ZR10 manual v1.7 p.14)
#   ZR30 / A8 mini: 4K stills
CAMERA_MODELS: Dict[str, str] = {
    'ZR10': '2K',
    'ZR30': '4K',
    'A8MINI': '4K',
    # Spec sheets not on hand for these - accept whatever they produce rather
    # than risk discarding good frames. Give them a real profile once verified.
    'A2MINI': 'ANY',
    'ZT6': 'ANY',
    'ZT30': 'ANY',
}

# Model auto-detection. CMD 0x02 returns a hardware-ID string whose first two
# characters are the model number in hex (ZR10 manual v1.7 p.44).
CMD_HARDWARE_ID = 0x02
HARDWARE_ID_TO_MODEL: Dict[int, str] = {
    0x6B: 'ZR10',
    0x73: 'A8MINI',
    0x75: 'A2MINI',
    0x78: 'ZR30',
    0x7A: 'ZT30',
    0x82: 'ZT6',
}

# 'AUTO' asks the camera at startup; the name below is the fallback used when
# the camera does not answer.
AUTO_CAMERA_MODEL = 'AUTO'
DEFAULT_CAMERA_MODEL = 'ZR10'

# Zoom the lens is parked at when the node starts, so every capture is taken at
# this magnification. ZR10 is 10x optical / 30x hybrid: above 10.0 is digital
# upscaling that adds no real detail, so keep this at 10.0 or below for
# detection work. Valid range per the SDK (CMD 0x0F, ZR10 manual v1.7 p.45) is
# 1.0 - 30.0. Set to None to leave whatever zoom the camera already holds.
CAPTURE_ZOOM_X = 5.0

# ── Focus ────────────────────────────────────────────────────────────────
# The lens is focused when the aircraft REACHES SURVEY ALTITUDE, not at node
# startup. _apply_capture_zoom() runs while the drone is still on the ground,
# and the autofocus bundled into CMD 0x0F therefore locks onto whatever is a
# few metres away. At CAPTURE_ZOOM_X = 5.0 the depth of field is a fraction of
# what it is at 1x, so that ground-level focus is nowhere near sharp once the
# subject is 30-100 m below.
#
#   'infinity' - drive the lens to its far stop once at altitude. Correct for
#                mapping: every subject is past the hyperfocal distance, and
#                unlike AF it cannot hunt or lock onto haze/low-contrast grass.
#   'auto'     - one-shot autofocus at the CENTRE of the frame.
#   'off'      - leave whatever focus the lens already holds.
FOCUS_MODE = 'infinity'

# Seconds to drive CMD_MANUAL_FOCUS 'far' before sending 'stop'. Overshooting
# is harmless -- the lens stops at its mechanical limit -- while undershooting
# leaves it short of infinity, so err long.
FOCUS_FAR_DRIVE_SECONDS = 3.0

# Autofocus touch point as a fraction of the frame (0.5, 0.5 = centre).
# CMD 0x04 takes PIXELS, so (0, 0) is the top-left CORNER, not the middle.
FOCUS_TOUCH_FRACTION = (0.5, 0.5)

# Frame the touch point is expressed in. SIYI touch coordinates are in the
# VIDEO stream's resolution (1080p), not the still-image resolution.
FOCUS_TOUCH_FRAME = (1920, 1080)

# Hold the camera lock this long after driving focus, so a capture arriving
# mid-rack waits for a settled lens instead of firing at a travelling one.
FOCUS_SETTLE_SECONDS = 2.5

# Re-run the focus action every N captures during a survey (0 disables), so a
# single dropped or failed attempt cannot cost the whole flight.
REFOCUS_EVERY_N_CAPTURES = 40

# Height AGL (metres) at which the climb-out focus fires. This deliberately
# does NOT reuse min_altitude_agl: that gate defaults to -13.716, so it is
# already satisfied on the ground and its "camera enabled" edge never triggers
# in a real flight. This one must be a height the aircraft genuinely climbs
# THROUGH, and high enough that the lens focuses on ground at survey distance.
FOCUS_AT_ALTITUDE_M = 15.0

# Re-arm the climb-out focus once the aircraft drops back below this height, so
# a second sortie in the same session focuses again. Kept well under
# FOCUS_AT_ALTITUDE_M so altitude noise around the trigger cannot re-arm it.
FOCUS_REARM_ALTITUDE_M = 5.0

PHOTO_RESOLUTIONS = {'4K': 0x00, '2K': 0x00, '2.7K': 0x00, '1080P': 0x00}
VERIFIED_RESOLUTIONS = set(CAMERA_MODELS.values())

# 0x0C func_type values (ZR10 manual v1.7 p.47). These are NOT a resolution
# selector: 1 toggles HDR and 2 starts/stops VIDEO RECORDING. Taking a still is
# always func_type 0, whatever the pixel size ends up being.
FUNC_TYPE_TAKE_PHOTO = 0
FUNC_TYPE_TOGGLE_HDR = 1
FUNC_TYPE_TOGGLE_RECORD = 2

# 55 66 01 01 00 00 00 0C <func_type=0> + CRC16
TAKE_PHOTO_COMMAND = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")

# Name -> packet, kept so callers stay unchanged; every entry is the same
# take-photo packet because the profile name does not affect the camera.
CAPTURE_COMMANDS = {name: TAKE_PHOTO_COMMAND for name in PHOTO_RESOLUTIONS}

# SIYI SDK protocol constants
SDK_STX = b'\x55\x66'

# SIYI SDK command IDs i found from the documentation
CMD_AUTO_FOCUS = 0x04
CMD_MANUAL_ZOOM_AF = 0x05
CMD_MANUAL_FOCUS = 0x06
CMD_GIMBAL_ROTATE = 0x07
CMD_GIMBAL_CENTER = 0x08
CMD_SYSTEM_INFO = 0x0A
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

# 0x0A ACK byte 4 (record_sta), SDK doc "Request Camera System Information".
# 2 means the camera firmware has not mounted the TF card - every 0x0C capture
# then answers 0x0B info_type=1 and no file is ever written.
RECORD_STA_NO_TF_CARD = 2
RECORD_STA_LABELS = {
    0: 'not recording',
    1: 'recording',
    2: 'NO TF CARD',
    3: 'video data loss (check TF card)',
}

# 0x0B info_type meanings (SDK doc "Function Feedback Response").
FUNCTION_FEEDBACK_LABELS = {
    0: 'photo captured successfully',
    1: 'photo failed - camera cannot see the TF card',
    2: 'HDR on',
    3: 'HDR off',
    4: 'video recording failed - camera cannot see the TF card',
    5: 'recording started',
    6: 'recording stopped',
}

STREAM_TYPE_LASER = 2

# Image specs
RESOLUTION_SPECS: Dict[str, Dict[str, int]] = {
    '4K': {'min_width': 3000, 'min_height': 1600, 'min_file_size': 50000},
    '2K': {'min_width': 2400, 'min_height': 1300, 'min_file_size': 30000},
    # Permissive profile for models whose native still size we have not
    # confirmed: still catches junk/truncated frames, imposes no upper claim.
    'ANY': {'min_width': 640, 'min_height': 480, 'min_file_size': 20000},
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
DEFAULT_RESOLUTION = CAMERA_MODELS[DEFAULT_CAMERA_MODEL]
DEFAULT_ROTATE_180 = False
