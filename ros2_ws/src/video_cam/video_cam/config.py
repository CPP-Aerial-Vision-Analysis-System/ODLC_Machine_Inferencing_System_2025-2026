#!/usr/bin/env python3
"""SIYI gimbal camera configuration constants (ZR10 / ZR30 / A8 mini).

Everything the camera node (siyi_node.py) needs to know about the hardware: addresses and ports,
SDK command ids, still-photo size per model, the zoom/focus routine, timing, storage, and (added
October 2026) what each model can do (CAMERA_CAPABILITIES), its field of view (CAMERA_HFOV_DEG),
how photos are oriented relative to the drone (DEFAULT_IMAGE_TOP_DIRECTION_DEG), and the photo
request queue (MAX_PENDING_CAPTURES ...). No ROS imports: unit tests can import it directly.
"""

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

# Zoom the lens is parked at for captures. Applied when DO_DIGICAM_CONTROL
# reaches the survey waypoint (do_digi_cam_trigger publishes "capture_setup"),
# NOT at node startup: racking on the ground bought nothing and left the
# autofocus bundled into CMD 0x0F locked a few metres away for the whole
# transit. ZR10 is 10x optical / 30x hybrid: above 10.0 is digital upscaling
# that adds no real detail, so keep this at 10.0 or below for detection work.
# Valid range per the SDK (CMD 0x0F, ZR10 manual v1.7 p.45) is 1.0 - 30.0.
# Set to None to leave whatever zoom the camera already holds.
CAPTURE_ZOOM_X = 5.0

# How the rack is confirmed. The camera's ack for CMD 0x0F often times out on
# a long travel even when the zoom lands, so success is judged by reading the
# magnification back to within ZOOM_TOLERANCE_X instead of trusting the ack.
# These cost flight time now that the rack happens at the survey waypoint with
# the capture timer already running, so keep the attempt count low.
ZOOM_ATTEMPTS = 3
ZOOM_SETTLE_SECONDS = 3.0
ZOOM_RETRY_DELAY_SECONDS = 2.0
ZOOM_TOLERANCE_X = 0.6

# ── Focus ────────────────────────────────────────────────────────────────
# The lens is focused at two points, never at node startup: when the aircraft
# REACHES SURVEY ALTITUDE (FOCUS_AT_ALTITUDE_M), and again immediately after
# the zoom rack on DO_DIGICAM_CONTROL. The second one is the authoritative
# pass -- CMD 0x0F carries its own autofocus, so racking to CAPTURE_ZOOM_X
# throws away whatever the climb focus set. At CAPTURE_ZOOM_X = 5.0 the depth
# of field is a fraction of what it is at 1x, so a focus that is even slightly
# off is visible once the subject is 30-100 m below.
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

# ── What each camera model can do (added October 2026) ───────────────────
# From SIYI's own support table (siyi_documentations/SIYI_Gimbal_Camera_SDK.pdf, UDP interface,
# printed pp.86-99): the A8 mini has a fixed-focus lens with DIGITAL zoom only, so it does not
# answer CMD 0x04 (auto focus), 0x06 (manual focus) or 0x18 (current zoom) -- "only supported by
# optical zoom cameras". Asking anyway costs a 2 s SDK timeout each time. Models without an entry
# keep every feature switched on (the behaviour before this table existed).
#   autofocus      CMD 0x04, used by the 'autofocus' command (the survey's 5 s autofocus loop)
#   manual_focus   CMD 0x06, used by FOCUS_MODE 'infinity'
#   zoom_readback  CMD 0x18, used to confirm the capture zoom rack
CAMERA_CAPABILITIES: Dict[str, Dict[str, bool]] = {
    'ZR10': {'autofocus': True, 'manual_focus': True, 'zoom_readback': True},
    'ZR30': {'autofocus': True, 'manual_focus': True, 'zoom_readback': True},
    'A8MINI': {'autofocus': False, 'manual_focus': False, 'zoom_readback': False},
}
DEFAULT_CAMERA_CAPABILITIES: Dict[str, bool] = {
    'autofocus': True, 'manual_focus': True, 'zoom_readback': True}


def camera_capabilities(model: str) -> Dict[str, bool]:
    """Which optional SDK features `model` answers (a copy; unknown models get everything)."""
    return dict(CAMERA_CAPABILITIES.get(str(model).upper(), DEFAULT_CAMERA_CAPABILITIES))


# Horizontal field of view of the STILL photos at 1x zoom, in degrees. Written into every photo's
# sidecar (photo_metadata.py) so main_controller can turn a pixel into metres on the ground.
# 81 deg is the A8 mini value the team's own notes use (detection/Documentation/DocForML.md) and
# what the Gazebo simulator renders for BOTH camera profiles. The ZR10 / ZR30 values are the same
# 81 deg until somebody measures them: photograph a ground object of known size from a known height
# and fix the number here (the aiming error grows with the error of this number, ~1 % per degree).
CAMERA_HFOV_DEG: Dict[str, float] = {'ZR10': 81.0, 'ZR30': 81.0, 'A8MINI': 81.0}
DEFAULT_HFOV_DEG = 81.0


def camera_hfov_deg(model: str) -> float:
    """Horizontal field of view (deg, at 1x zoom) of `model`; DEFAULT_HFOV_DEG if unknown."""
    return float(CAMERA_HFOV_DEG.get(str(model).upper(), DEFAULT_HFOV_DEG))


# Where the TOP of a photo points, clockwise from the drone's nose, when the gimbal looks straight
# down with its yaw centred (ROS parameter image_top_direction_deg of the siyi node):
#     0 = the nose. This is the simulator, and any camera head that tilts DOWN from looking ahead
#         (the A8 mini / ZR10 hanging under the nose, mounted the normal way round).
#    90 = the right side, 180 = the tail, 270 = the left side.
# CHECK ON THE REAL DRONE before trusting the drop aiming: put a marker 5 m in front of the drone,
# look straight down, take a photo: the marker must be at the TOP of the photo for 0. If the camera
# is mounted backwards use 180 (rotate_180 adds another 180 by itself).
DEFAULT_IMAGE_TOP_DIRECTION_DEG = 0.0

# ── Photo request queue (added October 2026) ─────────────────────────────
# Every /camera/trigger waits in a line (capture_queue.py) and is served as soon as the camera can
# take the photo; lens housekeeping (zoom racks, focus) no longer blocks photos for seconds.
MAX_PENDING_CAPTURES = 5            # requests that may wait; one more drops the OLDEST (logged)
CAPTURE_SERVICE_PERIOD = 0.1        # s, how often waiting requests are checked
CAPTURE_STATS_PERIOD = 30.0         # s, how often the photo statistics line is logged
# Manual /camera/command commands that talk to the camera directly wait at most this long for
# the camera (instead of freezing the whole node until the camera is free).
CAMERA_COMMAND_LOCK_TIMEOUT_SECONDS = 3.0

# ── Photo sidecar (added October 2026) ───────────────────────────────────
# A JSON file next to every photo with the drone's position, height and heading at the shutter and
# the camera's field of view / zoom / gimbal angles (photo_metadata.py). main_controller uses it to
# aim the payload drops at the target instead of at the photo's centre.
WRITE_PHOTO_SIDECAR = True
# The gimbal attitude (CMD 0x0D) is asked right before each shutter; a camera that does not answer
# within this time is skipped, and after this many failures in a row it is not asked again.
GIMBAL_ATTITUDE_TIMEOUT_SECONDS = 0.3
GIMBAL_ATTITUDE_MAX_FAILURES = 3
