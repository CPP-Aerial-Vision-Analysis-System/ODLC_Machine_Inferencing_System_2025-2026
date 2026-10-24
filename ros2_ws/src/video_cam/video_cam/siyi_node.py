#!/usr/bin/env python3
"""
The team's SIYI camera driver node: photos on /camera/trigger, lens commands on /camera/command.

siyi_node.py -- PURPOSE: the camera driver (ROS node name "siyi"). It talks to the SIYI gimbal
camera directly: the SIYI SDK over UDP 37260 (shutter, zoom, focus, gimbal) and the camera's HTTP
media server on port 82 (download the photo from its SD card). Every /camera/trigger
(std_msgs/Bool, sent once per second during the survey by mapping/do_digi_cam_trigger.py) becomes
one photo, saved as mapping_photos/"<lat> , <lon>.jpg" with the drone's GPS position at the
shutter, where the detector (detection/new_od.py) picks it up. /camera/command (std_msgs/String)
runs lens and gimbal commands ("capture_setup", "autofocus", "gimbal_set_angles 0,-90", ...).

Changed October 2026 (bug "dropped photos", plus the drop-aiming fix):
  * Photo requests WAIT IN A LINE (capture_queue.py) and are served as soon as the camera can take
    a photo. Before, a trigger that came while the lens worker zoomed or focused was thrown away:
    49 of 83 triggers in one simulator run.
  * Photos have priority over lens housekeeping. Zoom racks and focus passes hold the camera lock
    only while an SDK command is exchanged (milliseconds), not through their 2.5-3 s settle times,
    so a photo can be taken in between. Photos taken while the zoom moves are marked as such.
  * The node itself never waits for the camera lock while handling messages (it used to freeze
    for seconds, so GPS updates were late and photo names used an old position).
  * Commands a model cannot do are not sent: an A8 mini has no autofocus / manual focus / zoom
    readback (config.CAMERA_CAPABILITIES), so the survey's autofocus loop is skipped on it
    instead of costing a 2 s timeout every 5 s.
  * The GPS name is taken right before the shutter, and a JSON "sidecar" (photo_metadata.py) is
    written next to every photo: position, height above home, heading, field of view, zoom,
    gimbal angles and the photo's orientation relative to the drone (parameter
    image_top_direction_deg). main_controller uses it to aim the payload drops at the target.
"""

from video_cam.logging_utils import configure_console_format, install_wallclock_logging
configure_console_format()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

import json
import os
import cv2
import time
import numpy as np
from threading import Lock, Event, Thread
from typing import Optional, Dict, Any, List

from video_cam.storage_manager import get_ros2_ws_directory

from .config import (
    NODE_LOOP_PERIOD,
    DEFAULT_USE_REAL_CAMERA,
    DEFAULT_MIN_ALTITUDE_AGL,
    DEFAULT_RESOLUTION,
    DEFAULT_CAMERA_MODEL,
    AUTO_CAMERA_MODEL,
    CAMERA_MODELS,
    DEFAULT_ROTATE_180,
    CAMERA_IP,
    CONTROL_PORT,
    MEDIA_PORT,
    HTTP_TIMEOUT_SECONDS,
    CAPTURE_TIMEOUT_SECONDS,
    MIN_FREE_SPACE_MB,
    PHOTO_RESOLUTIONS,
    CAPTURE_ZOOM_X,
    ZOOM_ATTEMPTS,
    ZOOM_SETTLE_SECONDS,
    ZOOM_RETRY_DELAY_SECONDS,
    ZOOM_TOLERANCE_X,
    FOCUS_MODE,
    FOCUS_FAR_DRIVE_SECONDS,
    FOCUS_TOUCH_FRACTION,
    FOCUS_TOUCH_FRAME,
    FOCUS_SETTLE_SECONDS,
    REFOCUS_EVERY_N_CAPTURES,
    FOCUS_AT_ALTITUDE_M,
    FOCUS_REARM_ALTITUDE_M,
    DEFAULT_IMAGE_TOP_DIRECTION_DEG,
    MAX_PENDING_CAPTURES,
    CAPTURE_SERVICE_PERIOD,
    CAPTURE_STATS_PERIOD,
    CAMERA_COMMAND_LOCK_TIMEOUT_SECONDS,
    WRITE_PHOTO_SIDECAR,
    GIMBAL_ATTITUDE_TIMEOUT_SECONDS,
    GIMBAL_ATTITUDE_MAX_FAILURES,
    camera_capabilities,
    camera_hfov_deg,
)
from .camera_interface import CameraInterface, CameraConnectionError
from .storage_manager import StorageManager
from .pipeline_orchestrator import PipelineOrchestrator
from .capture_queue import CaptureRequestQueue
from .photo_metadata import build_photo_metadata, sidecar_path_for, write_sidecar

# /camera/command commands that only QUEUE work (for the photo queue or the lens worker thread).
# They are handled without taking the camera lock, so they can never freeze this node.
QUEUED_COMMANDS = {'capture', 'autofocus', 'focus', 'focus_infinity', 'capture_setup',
                   'zoom_capture'}


class SIYINode(Node):
    """ROS2 node wrapper for SIYI camera pipeline."""
    
    def __init__(self):
        super().__init__('siyi')
        install_wallclock_logging(self)

        # Parameters
        self.declare_parameter('use_real_camera', DEFAULT_USE_REAL_CAMERA)
        self.declare_parameter('min_altitude_agl', DEFAULT_MIN_ALTITUDE_AGL)
        self.declare_parameter('camera_ip', CAMERA_IP)
        self.declare_parameter('ctrl_port', CONTROL_PORT)
        self.declare_parameter('media_port', MEDIA_PORT)
        self.declare_parameter('http_timeout_sec', HTTP_TIMEOUT_SECONDS)
        self.declare_parameter('capture_timeout_sec', CAPTURE_TIMEOUT_SECONDS)
        self.declare_parameter('min_free_space_mb', MIN_FREE_SPACE_MB)
        # Which gimbal is bolted on. Still-photo size is fixed by the sensor
        # (ZR10 = 2K, ZR30 = 4K), so the model decides the validation profile.
        self.declare_parameter('camera_model', AUTO_CAMERA_MODEL)
        # Leave empty to derive the profile from camera_model.
        self.declare_parameter('resolution', '')
        self.declare_parameter('rotate_180', DEFAULT_ROTATE_180)
        # Where the top of a photo points, clockwise from the drone's nose, when the gimbal looks
        # straight down (0 = nose, the simulator; config.DEFAULT_IMAGE_TOP_DIRECTION_DEG says how
        # to check it on the real drone). Written into every photo's sidecar for the drop aiming.
        # dynamic_typing: whole numbers are accepted too (`:=90` as well as `:=90.0`). With a plain
        # 0.0 default ROS 2 refuses an INTEGER and this node would not start (no photos at all).
        self.declare_parameter('image_top_direction_deg', DEFAULT_IMAGE_TOP_DIRECTION_DEG,
                               ParameterDescriptor(dynamic_typing=True))

        self.use_real_camera = self.get_parameter('use_real_camera').value
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.media_port = self.get_parameter('media_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.capture_timeout = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
        # Empty means "follow the camera model"; anything else pins the profile.
        self.resolution_override = str(self.get_parameter('resolution').value or '').upper()
        if self.resolution_override and self.resolution_override not in PHOTO_RESOLUTIONS:
            self.get_logger().warn(
                f"Invalid resolution '{self.resolution_override}', following the "
                f"camera model instead. Valid: {sorted(PHOTO_RESOLUTIONS)}")
            self.resolution_override = ''
        self.resolution = self.resolution_override
        self.rotate_180 = self.get_parameter('rotate_180').value
        # The saved photo is turned another 180 deg when rotate_180 is on.
        top = self.get_parameter('image_top_direction_deg').value
        if isinstance(top, bool) or not isinstance(top, (int, float)):
            # Not a number (e.g. `:=true` or a word): say so, and keep taking photos.
            self.get_logger().error(
                f"image_top_direction_deg={top!r} is not a number: using the default "
                f"{DEFAULT_IMAGE_TOP_DIRECTION_DEG:g}")
            top = DEFAULT_IMAGE_TOP_DIRECTION_DEG
        self.image_top_direction_deg = (
            float(top) + (180.0 if self.rotate_180 else 0.0)) % 360.0

        # 'AUTO' (the default) asks the camera itself once it is connected,
        # below. An explicit model here pins it and skips detection.
        self.camera_model = str(self.get_parameter('camera_model').value).upper().replace('_', '').replace(' ', '')
        self.auto_detect_model = self.camera_model in ('', AUTO_CAMERA_MODEL)
        if not self.auto_detect_model and self.camera_model not in CAMERA_MODELS:
            self.get_logger().warn(
                f"Unknown camera_model '{self.camera_model}', detecting instead. "
                f"Known: {sorted(CAMERA_MODELS)}")
            self.auto_detect_model = True

        if self.auto_detect_model:
            self.camera_model = DEFAULT_CAMERA_MODEL  # provisional until detection
        self._apply_camera_model(self.camera_model, announce=not self.auto_detect_model)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        self.disk_status_pub = self.create_publisher(Float64, '/camera/disk_free_mb', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        # Compass heading at the shutter goes into each photo's sidecar (drop aiming).
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/camera/command', self.camera_command_callback, 10)
        if not self.use_real_camera:
            self.create_subscription(Image, '/camera/image', self.sim_image_callback, 1)

        # State
        self.camera_enabled = True
        self.camera_control_lock = Lock()
        # Photo requests wait here until the camera can take them (oldest first, bounded).
        self.capture_queue = CaptureRequestQueue(MAX_PENDING_CAPTURES)
        # Set from the moment a capture is started until its shutter + SD-index phases are done:
        # only one capture may be in those phases at a time (its download may overlap the next).
        self._capture_active = Event()
        self.latest_image_msg: Optional[Image] = None
        self.latest_gps = None
        self.latest_gps_time: Optional[float] = None
        self.latest_rel_alt: Optional[float] = None
        self.latest_rel_alt_time: Optional[float] = None
        self.latest_heading: Optional[float] = None
        self.latest_heading_time: Optional[float] = None
        self.bridge = CvBridge()
        self._photos_saved = 0
        self._last_stats_text = ''

        # Lens state. Zoom and focus share ONE worker thread so they can
        # never overlap, and neither runs at startup: both are driven by the
        # flight (climb-out, then DO_DIGICAM_CONTROL). See _request_lens_setup.
        self._lens_thread: Optional[Thread] = None
        self._captures_since_focus = 0
        self._climb_focus_done = False
        # What the photos' sidecars say about the zoom. Cameras start at 1x; after a rack this is
        # the camera's own readback (or the commanded value on cameras that cannot report it).
        self._zoom_x = 1.0
        self._zoom_source = 'assumed (cameras start at 1x)'
        self._zoom_changing = False
        self._skip_logged = set()            # "not supported by this camera" lines already logged
        # Gimbal attitude (CMD 0x0D) for the sidecar: switched off after repeated silence.
        self._gimbal_attitude_ok = True
        self._gimbal_attitude_failures = 0

        # Components
        workspace_root = self._find_ros2_workspace()
        self.storage = StorageManager(workspace_root, logger=self.get_logger())

        if self.use_real_camera:
            self.camera = CameraInterface(
                camera_ip=self.camera_ip,
                ctrl_port=self.ctrl_port,
                media_port=self.media_port,
                http_timeout=self.http_timeout,
                logger=self.get_logger()
            )
            if self.auto_detect_model:
                detected = self.camera.detect_model()
                if detected:
                    self._apply_camera_model(detected)
                else:
                    self.get_logger().warn(
                        f"Camera model detection failed - assuming "
                        f"{self.camera_model}. Pin it with -p camera_model:=<model> "
                        f"if that is wrong.")
                    self._apply_camera_model(self.camera_model)

            self.pipeline = PipelineOrchestrator(
                camera=self.camera,
                storage=self.storage,
                rotate_180=self.rotate_180,
                logger=self.get_logger()
            )
            self.pipeline.set_resolution(self.resolution)
            self.pipeline.initialize_sd_card()
            # No zoom here. The lens is racked to CAPTURE_ZOOM_X only when
            # DO_DIGICAM_CONTROL arrives at the survey waypoint; see
            # _apply_capture_zoom.
        else:
            self.camera = None
            self.pipeline = None
            self.get_logger().info("Simulation mode: Waiting for images on /camera/image...")
            self._send_status("Simulation camera initialized")
        
        # Main loop timer (disk status, 1 Hz)
        self.pipeline_timer = self.create_timer(NODE_LOOP_PERIOD, self._pipeline_loop)
        # Photo queue service: a waiting request starts within CAPTURE_SERVICE_PERIOD of the camera
        # becoming free (the old 1 Hz loop could also merge two triggers into one photo).
        self.capture_timer = self.create_timer(CAPTURE_SERVICE_PERIOD, self._service_capture_queue)
        self.stats_timer = self.create_timer(CAPTURE_STATS_PERIOD, self._log_capture_stats)

        mode = 'SIMULATION' if not self.use_real_camera else 'REAL CAMERA'
        caps = camera_capabilities(self.camera_model)
        missing = [name for name, ok in caps.items() if not ok]
        self.get_logger().info(
            f"SIYI pipeline initialized ({mode}); photo queue holds {MAX_PENDING_CAPTURES} "
            f"requests; photo top points {self.image_top_direction_deg:.0f} deg clockwise from "
            f"the nose" + (f"; {self.camera_model} cannot do: {', '.join(missing)}"
                           if missing else ""))
    
    def _apply_camera_model(self, model: str, announce: bool = True) -> None:
        """Adopt `model` and derive the image-validation profile from it.

        An explicit `resolution` parameter always wins; otherwise the profile
        follows the camera's native still size (ZR10 = 2K, A8 mini/ZR30 = 4K).
        """
        self.camera_model = model
        self.native_resolution = CAMERA_MODELS[model]
        self.resolution = self.resolution_override or self.native_resolution

        # May run before the pipeline exists (initial parameter parsing).
        pipeline = getattr(self, 'pipeline', None)
        if pipeline is not None:
            pipeline.set_resolution(self.resolution)

        if announce:
            suffix = (f" [resolution pinned to {self.resolution} by parameter]"
                      if self.resolution_override else "")
            self.get_logger().info(
                f"Camera model: {self.camera_model} "
                f"(native stills: {self.native_resolution}){suffix}")

    def _capabilities(self) -> Dict[str, bool]:
        """What the current camera model answers (config.CAMERA_CAPABILITIES)."""
        return camera_capabilities(self.camera_model)

    def _log_skip_once(self, key: str, text: str) -> None:
        """Log a "not supported by this camera" message only the first time."""
        if key not in self._skip_logged:
            self._skip_logged.add(key)
            self.get_logger().info(text)

    def _set_zoom(self, zoom_x: Optional[float], source: str) -> None:
        """Remember the zoom the next photos' sidecars report."""
        if zoom_x is not None and zoom_x > 0:
            self._zoom_x = float(zoom_x)
            self._zoom_source = source

    def _apply_capture_zoom(self, reason: str) -> bool:
        """Park the lens at CAPTURE_ZOOM_X so every photo is taken there.

        Driven by DO_DIGICAM_CONTROL (do_digi_cam_trigger publishes
        "capture_setup" on arrival at the survey waypoint), never by node
        startup: the lens needs seconds to travel and refocus, and racking it
        on the ground left the autofocus bundled into CMD 0x0F locked onto
        whatever sat a few metres from the aircraft.

        The camera lock is held only while each SDK command is exchanged; the
        settle time is spent WITHOUT it, so photos keep being taken during the
        rack (October 2026: holding the lock here dropped every photo request
        for the whole rack). Photos taken meanwhile are marked "zoom changing"
        in their sidecar, so main_controller does not trust their scale.

        No-ops when the lens already sits at the target - DigiCamCtrl repeats
        on every pass through the waypoint and must not re-rack each time.

        A falsy return from absolute_zoom_autofocus is deliberately not treated
        as failure - the SDK ack for CMD 0x0F often times out on a long travel
        while the zoom itself still happens - so the result is confirmed by
        reading the zoom back instead (cameras that cannot report their zoom,
        like the A8 mini, get one command and the commanded value is assumed).
        """
        if CAPTURE_ZOOM_X is None:
            self.get_logger().info("CAPTURE_ZOOM_X is None - leaving zoom as-is")
            return False

        if not 1.0 <= CAPTURE_ZOOM_X <= 30.0:
            self.get_logger().error(
                f"CAPTURE_ZOOM_X={CAPTURE_ZOOM_X} is outside the SDK range "
                f"1.0-30.0 - leaving zoom untouched.")
            return False

        can_read_zoom = self._capabilities()['zoom_readback']
        current = None
        if can_read_zoom:
            try:
                with self.camera_control_lock:
                    current = self.camera.get_current_zoom_magnification()
            except (CameraConnectionError, OSError, ValueError) as exc:
                self.get_logger().warn(
                    f"Zoom readback failed ({exc}) - racking anyway")
                current = None
            self._set_zoom(current, 'readback')

        if (current is not None
                and abs(current - CAPTURE_ZOOM_X) <= ZOOM_TOLERANCE_X):
            self.get_logger().info(
                f"Zoom already at {current}x ({reason}) - no rack needed")
            return True

        self._zoom_changing = True
        try:
            for attempt in range(1, ZOOM_ATTEMPTS + 1):
                try:
                    with self.camera_control_lock:
                        self.camera.absolute_zoom_autofocus(CAPTURE_ZOOM_X)
                except (CameraConnectionError, OSError, ValueError) as exc:
                    self.get_logger().warn(f"Zoom attempt {attempt} errored: {exc}")
                    time.sleep(ZOOM_RETRY_DELAY_SECONDS)
                    continue

                time.sleep(ZOOM_SETTLE_SECONDS)       # no lock held: photos continue

                if not can_read_zoom:
                    # This camera cannot report its zoom (A8 mini: no CMD 0x18). It does take
                    # CMD 0x0F, so assume the commanded value instead of retrying blindly.
                    self._set_zoom(CAPTURE_ZOOM_X,
                                   'commanded (this camera cannot report its zoom)')
                    self.get_logger().info(
                        f"Zoom set to {CAPTURE_ZOOM_X}x (commanded; {self.camera_model} cannot "
                        f"report its zoom, {reason})")
                    return True

                try:
                    with self.camera_control_lock:
                        actual = self.camera.get_current_zoom_magnification()
                except (CameraConnectionError, OSError, ValueError) as exc:
                    self.get_logger().warn(f"Zoom readback failed ({exc})")
                    actual = None
                self._set_zoom(actual, 'readback')
                if (actual is not None
                        and abs(actual - CAPTURE_ZOOM_X) <= ZOOM_TOLERANCE_X):
                    self.get_logger().info(
                        f"Capture zoom set to {actual}x "
                        f"(asked {CAPTURE_ZOOM_X}x, {reason})")
                    self._send_status(f"Zoom set to {actual}x")
                    return True
                self.get_logger().warn(
                    f"Zoom attempt {attempt}: asked {CAPTURE_ZOOM_X}x, "
                    f"camera reports {actual}x")
        finally:
            self._zoom_changing = False

        self.get_logger().warn(
            f"Could not confirm {CAPTURE_ZOOM_X}x after {ZOOM_ATTEMPTS} attempts "
            f"- continuing. Zoom readback on this camera is unreliable; check "
            f"the video feed.")
        return False

    def _focus_touch_point(self) -> tuple:
        """Centre of the frame in the SDK's touch coordinates.

        CMD 0x04 takes a touch point in PIXELS of the video stream, so (0, 0)
        is the top-left CORNER, not the middle. Focusing the corner of a
        downward-looking frame locks the lens onto whatever happens to sit at
        the edge of the strip - often sky, rotor or horizon rather than ground.
        """
        fx, fy = FOCUS_TOUCH_FRACTION
        width, height = FOCUS_TOUCH_FRAME
        return int(width * fx), int(height * fy)

    def _check_climb_focus(self, current_alt: float) -> None:
        """Focus once the aircraft has actually climbed to survey height.

        A first pass so the transit frames are usable at whatever zoom the
        lens happens to hold. It is NOT the authoritative one: the focus queued
        behind the rack on DO_DIGICAM_CONTROL is, because CMD 0x0F carries its
        own autofocus and racking to CAPTURE_ZOOM_X discards whatever this set.

        Deliberately NOT hung off the camera_enabled edge in altitude_callback:
        min_altitude_agl defaults to -13.716, so that gate is already satisfied
        on the ground and its rising edge never fires in a real flight.
        """
        if current_alt >= FOCUS_AT_ALTITUDE_M:
            if not self._climb_focus_done:
                self._climb_focus_done = True
                self._request_focus(
                    f"climbed through {FOCUS_AT_ALTITUDE_M:.0f}m AGL "
                    f"(now {current_alt:.1f}m)")
        elif current_alt < FOCUS_REARM_ALTITUDE_M:
            # Back on the ground: re-arm so a second sortie focuses again.
            self._climb_focus_done = False

    def _lens_busy(self) -> bool:
        """True while a zoom rack or focus is in flight on the lens worker."""
        return self._lens_thread is not None and self._lens_thread.is_alive()

    def _focus_supported(self, mode: str) -> bool:
        """Can this camera do a focus pass of `mode` ('infinity' needs 0x06, 'auto' needs 0x04)?"""
        caps = self._capabilities()
        if mode == 'infinity':
            return caps['manual_focus']
        return caps['autofocus']

    def _request_lens_setup(self, reason: str, zoom: bool = False,
                            focus: bool = True, mode: Optional[str] = None,
                            touch: Optional[tuple] = None) -> bool:
        """Queue a zoom rack and/or a focus onto the single lens worker thread.

        Never blocks the caller and never joins: this is called from
        subscription callbacks, and the worker takes the camera lock itself.

        Zoom and focus share ONE thread rather than getting one each. They have
        to run in that order and must never overlap: CMD 0x0F carries its own
        autofocus, so a focus that finished first would simply be thrown away
        by the rack that followed it.

        A focus the camera cannot do (config.CAMERA_CAPABILITIES, e.g. any focus
        on an A8 mini) is left out instead of being sent and timing out.
        """
        if not self.use_real_camera or self.camera is None:
            return False

        mode = (mode or FOCUS_MODE).lower()
        if focus and mode == 'off':
            self.get_logger().info(
                f"Focus skipped ({reason}): FOCUS_MODE is 'off'")
            focus = False
        if focus and not self._focus_supported(mode):
            self._log_skip_once(
                f'focus-{mode}',
                f"Focus '{mode}' skipped: the {self.camera_model} does not support it "
                f"(fixed-focus lens; SIYI SDK table). Not sent again this flight.")
            focus = False

        if not (zoom or focus):
            return False

        if self._lens_busy():
            self.get_logger().info(
                f"Lens work already in progress - skipping request ({reason})")
            return False

        self._lens_thread = Thread(
            target=self._lens_worker, args=(reason, zoom, focus, mode, touch),
            daemon=True)
        self._lens_thread.start()
        return True

    def _request_focus(self, reason: str, mode: Optional[str] = None,
                       touch: Optional[tuple] = None) -> bool:
        """Focus-only shorthand for _request_lens_setup."""
        return self._request_lens_setup(reason, zoom=False, focus=True,
                                        mode=mode, touch=touch)

    def _lens_worker(self, reason: str, zoom: bool, focus: bool, mode: str,
                     touch: Optional[tuple]) -> None:
        """Rack the lens, then focus it - in that order, never concurrently."""
        if zoom:
            self._apply_capture_zoom(reason)
        if focus:
            self._focus_once(reason, mode, touch)

    def _focus_once(self, reason: str, mode: str,
                    touch: Optional[tuple]) -> None:
        """Drive the lens, then let it settle.

        The camera lock is held only while each SDK command is exchanged. The
        far-drive and the settle time run WITHOUT it, so photos keep being
        taken (October 2026: holding the lock through the settle dropped about
        3 photos out of every 5 during the survey's 5 s autofocus loop). A photo
        taken during a focus pass can be a little soft; a dropped photo is a
        hole in the search.
        """
        ok = False
        try:
            if mode == 'infinity':
                self.get_logger().info(
                    f"Focusing to infinity ({reason}): driving far for "
                    f"{FOCUS_FAR_DRIVE_SECONDS}s")
                with self.camera_control_lock:
                    ok = self.camera.manual_focus(1)       # 1 = far
                time.sleep(FOCUS_FAR_DRIVE_SECONDS)
                with self.camera_control_lock:
                    self.camera.manual_focus(0)            # 0 = stop
            else:
                x, y = touch if touch else self._focus_touch_point()
                self.get_logger().info(
                    f"Autofocus ({reason}) at touch point ({x}, {y})")
                with self.camera_control_lock:
                    ok = self.camera.auto_focus(touch_x=x, touch_y=y)

            time.sleep(FOCUS_SETTLE_SECONDS)

            self._captures_since_focus = 0
            if ok:
                self.get_logger().info(f"Focus set ({mode}) - {reason}")
                self._send_status(f"Focus set ({mode})")
            else:
                self.get_logger().warn(
                    f"Camera did not ack the focus command ({mode}); the lens "
                    f"may still have moved - check the first captures.")
        except (CameraConnectionError, OSError, ValueError) as exc:
            self.get_logger().error(f"Focus attempt failed ({reason}): {exc}")

    def _find_ros2_workspace(self) -> str:
        ros2_ws_dir = get_ros2_ws_directory()
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        return video_cam_dir

    def _pipeline_loop(self):
        # Main loop (1 Hz): disk status. Photo requests are served by _service_capture_queue.
        self._publish_disk_status()
    
    def _service_capture_queue(self):
        """Start the oldest waiting photo request as soon as the camera can take it.

        Runs every CAPTURE_SERVICE_PERIOD and right after each trigger. Only one
        capture may be in its shutter + SD-index phases at a time; its HTTP
        download runs in parallel with the next shutter (pipelined capture).
        Lens work (zoom rack, focus) does NOT hold photos back any more.
        """
        if self.capture_queue.pending == 0:
            return
        if not self.camera_enabled:
            # Below min_altitude_agl photos are not wanted: forget the requests.
            n = self.capture_queue.clear()
            self.get_logger().debug(f"Camera disabled (altitude) - ignoring {n} photo request(s)")
            return
        if self.use_real_camera:
            if self._capture_active.is_set():
                return                          # the previous shutter/index is still running
            trigger_time = self.capture_queue.pop()
            if trigger_time is None:
                return
            # Set HERE (not in the thread) so the next timer tick cannot start a second capture
            # before the thread has begun.
            self._capture_active.set()
            capture_thread = Thread(target=self._execute_real_camera_capture,
                                    args=(trigger_time,), daemon=True)
            capture_thread.start()
        else:
            # Simulation mode: save current image
            self.capture_queue.pop()
            self._execute_simulation_capture()
    
    def _log_capture_stats(self, force: bool = False):
        """Log the photo statistics (only when they changed, unless force)."""
        text = (f"Photo stats: {self.capture_queue.stats_text()}; "
                f"{self._photos_saved} photos saved")
        if force or text != self._last_stats_text:
            self._last_stats_text = text
            self.get_logger().info(text)

    def _query_gimbal_attitude(self) -> Optional[Dict[str, float]]:
        """The gimbal's own yaw/pitch/roll (CMD 0x0D) for the photo's sidecar, or None.

        Called with the camera lock held, right before the shutter. A camera that
        stays silent is asked again a few times, then not any more this flight.
        """
        if not self._gimbal_attitude_ok:
            return None
        try:
            data = self.camera.request_gimbal_attitude(timeout=GIMBAL_ATTITUDE_TIMEOUT_SECONDS)
        except (CameraConnectionError, OSError, ValueError):
            data = None
        if data is None:
            self._gimbal_attitude_failures += 1
            if self._gimbal_attitude_failures >= GIMBAL_ATTITUDE_MAX_FAILURES:
                self._gimbal_attitude_ok = False
                self.get_logger().warn(
                    "The camera did not report its gimbal attitude (CMD 0x0D) "
                    f"{self._gimbal_attitude_failures} times in a row - not asking any more; "
                    "photo sidecars will have no gimbal angles (main_controller then assumes "
                    "the camera looks straight down).")
            return None
        self._gimbal_attitude_failures = 0
        return {'yaw_deg': data['yaw_deg'], 'pitch_deg': data['pitch_deg'],
                'roll_deg': data['roll_deg'], 'source': 'camera 0x0D'}

    def _capture_metadata(self, gps_filename: Optional[str], trigger_time: float) -> dict:
        """Everything the drone and camera were doing right now, for the photo's sidecar."""
        now = time.time()
        fix = self.latest_gps

        def age(t):
            return None if t is None else now - t
        lat = lon = alt = None
        if fix is not None:
            lat, lon, alt = fix.latitude, fix.longitude, fix.altitude
        return build_photo_metadata(
            image_name=gps_filename, shutter_unix_s=now, trigger_unix_s=trigger_time,
            lat=lat, lon=lon, alt_msl_m=alt, fix_age_s=age(self.latest_gps_time),
            rel_alt_m=self.latest_rel_alt, rel_alt_age_s=age(self.latest_rel_alt_time),
            heading_deg=self.latest_heading, heading_age_s=age(self.latest_heading_time),
            camera_model=self.camera_model, hfov_deg=camera_hfov_deg(self.camera_model),
            zoom_x=self._zoom_x, zoom_source=self._zoom_source,
            zoom_changing=self._zoom_changing,
            image_top_direction_deg=self.image_top_direction_deg, rotate_180=self.rotate_180,
            gimbal=self._query_gimbal_attitude())

    def _execute_real_camera_capture(self, trigger_time: Optional[float] = None):
        """Execute real camera capture pipeline with pipelined phases.

        Phases 1+2 (UDP shutter + SD index) run under camera_control_lock so
        they are serialized with other camera commands. Phase 3 (HTTP download
        on a separate port) runs OUTSIDE the lock, so the NEXT capture's
        shutter and the previous capture's download can overlap — cutting
        back-to-back capture latency roughly in half.
        """
        if trigger_time is None:
            trigger_time = time.time()
        try:
            try:
                # Phases 1+2: UDP shutter + SD card indexing. Hold the camera
                # control lock here so gimbal/zoom commands can't race with
                # the UDP capture command.
                with self.camera_control_lock:
                    # Snapshot a GPS-based filename right BEFORE the shutter so
                    # that phase 3 saves the downloaded image directly under its
                    # final "<lat> , <lon>.jpg" name (new_od must never see it
                    # under its SD-card name). Taken INSIDE the lock (October
                    # 2026): taken before it, a capture that had to wait for the
                    # lens got a name seconds older than the photo (15 m off).
                    gps_filename = self._generate_gps_filename()
                    metadata = (self._capture_metadata(gps_filename, trigger_time)
                                if WRITE_PHOTO_SIDECAR else None)
                    file_info = self.pipeline.capture_and_index()
            finally:
                # Shutter + index are over: the next photo may start now (even if this one failed).
                self._capture_active.clear()

            if gps_filename is None:
                self.get_logger().warn( "No valid GPS info /mavros/global_position/global; using the SD name")
                self._send_status("WARN: No GPS fix - image will not have lat/lon name")

            if file_info is None:
                self._send_status("FAILED: Capture shutter/index error")
                self._publish_camera_status("FAILURE: Capture shutter/index error")
                return

            # Refocus periodically so one dropped or failed focus attempt
            # cannot cost the rest of the flight. Requested outside the lock
            # above; the worker takes it again for each SDK command only.
            self._captures_since_focus += 1
            if (REFOCUS_EVERY_N_CAPTURES > 0
                    and self._captures_since_focus >= REFOCUS_EVERY_N_CAPTURES):
                self._request_focus(
                    f"{self._captures_since_focus} captures since last focus")

            # The sidecar goes next to the photo BEFORE the photo appears, so
            # whoever reacts to the photo (new_od -> main_controller) finds it.
            if metadata is not None:
                final_name = gps_filename or file_info.get('name', '')
                if final_name:
                    metadata['image_name'] = final_name
                    photo_path = os.path.join(self.storage.get_mapping_dir(), final_name)
                    path = sidecar_path_for(photo_path)
                    if not write_sidecar(path, metadata):
                        self.get_logger().warn(f"Could not write the photo sidecar {path}")

            # Phase 3: HTTP download on port 82. Intentionally NOT holding
            # camera_control_lock here so that (a) gimbal/zoom commands can
            # run concurrently and (b) the next capture's phases 1+2 can
            # overlap with this download.
            result = self.pipeline.download_and_save(
                file_info, filename_override=gps_filename
            )

            if result is None:
                self._send_status("FAILED: Capture download error")
                self._publish_camera_status("FAILURE: Capture download error")
                return

            saved_path, img = result
            self._photos_saved += 1
            stats = self.pipeline.get_stats()
            self._send_status(
                f"SUCCESS: Captured {stats['resolution']} image #{stats['photo_count']}")
            # self._publish_camera_status(
            #     f"SUCCESS: {stats['resolution']} image captured")

            # Publish the in-memory decoded ndarray directly — no disk
            # re-read, no directory scan, no rename.
            self._publish_captured_image(img, saved_path)

        except Exception as e:
            self.get_logger().error(f"Pipeline error: {e}")
            self._send_status(f"FAILED: {e}")
            self._publish_camera_status(f"FAILURE: {e}")

    @staticmethod
    def _result_payload(ok: bool, **kwargs: Any) -> Dict[str, Any]:
        payload: Dict[str, Any] = {'ok': ok}
        payload.update(kwargs)
        return payload

    @staticmethod
    def _split_csv(parameter: str, expected_len: int) -> List[str]:
        parts = [item.strip() for item in parameter.split(',') if item.strip()]
        if len(parts) != expected_len:
            raise ValueError(
                f"Expected {expected_len} CSV values, got {len(parts)}"
            )
        return parts

    @staticmethod
    def _parse_switch(value: str) -> bool:
        normalized = value.strip().lower()
        if normalized in {'on', '1', 'true', 'enable', 'enabled'}:
            return True
        if normalized in {'off', '0', 'false', 'disable', 'disabled'}:
            return False
        raise ValueError("Switch value must be on/off (or true/false, 1/0)")

    def gps_cb(self, msg):
        """Callback to store the latest GPS data."""
        self.latest_gps = msg
        self.latest_gps_time = time.time()

    def heading_cb(self, msg: Float64):
        """Latest compass heading (degrees, 0 = north), for the photo sidecars."""
        self.latest_heading = float(msg.data)
        self.latest_heading_time = time.time()

    def _execute_camera_command(self, command: str, parameter: str) -> Dict[str, Any]:
        if not self.use_real_camera or self.camera is None:
            return self._result_payload(
                False,
                error='Camera command service requires use_real_camera=true',
            )

        cmd = command.strip().lower().replace('-', '_')
        param = parameter.strip()

        # Capture command goes into the same photo queue as /camera/trigger.
        if cmd == 'capture':
            resolution = self.resolution
            if param:
                resolution = param.upper()
                if resolution not in PHOTO_RESOLUTIONS:
                    raise ValueError(
                        f"Invalid resolution '{resolution}'. Valid: {sorted(PHOTO_RESOLUTIONS.keys())}"
                    )
                if self.pipeline is not None:
                    if self.pipeline.is_busy():
                        # Changing the validation profile mid-capture would mislabel that photo.
                        return self._result_payload(False, action='capture',
                                                    error='Pipeline is busy')
                    self.pipeline.set_resolution(resolution)

            self.capture_queue.push(time.time())
            self._service_capture_queue()
            return self._result_payload(True, action='capture', queued=True,
                                        resolution=resolution)

        # Focus commands are QUEUED onto the focus worker rather than run
        # inline, and are handled BEFORE the busy guard below. During a survey
        # the pipeline is busy almost continuously at 1 Hz, so an inline focus
        # request would be rejected for the whole flight - which is exactly why
        # the autofocus published by do_digi_cam_trigger never took effect.
        if cmd in {'autofocus', 'focus'}:
            touch = None
            if param:
                x_raw, y_raw = self._split_csv(param, 2)
                touch = (int(x_raw), int(y_raw))
            mode = 'auto' if cmd == 'autofocus' else FOCUS_MODE
            if not self._focus_supported(mode):
                # Not an error: this camera simply has no such focus (A8 mini). Say so once.
                self._log_skip_once(
                    f'focus-{mode}',
                    f"'{cmd}' skipped: the {self.camera_model} does not support focus '{mode}' "
                    f"(fixed-focus lens; SIYI SDK table). Ignored for the rest of the flight.")
                return self._result_payload(True, action=cmd, mode=mode, queued=False,
                                            skipped='not supported by this camera')
            if self._lens_busy():
                # The lens worker is busy (e.g. the capture_setup rack): skip this pass, the
                # next one comes with the next autofocus request. Not an error.
                return self._result_payload(True, action=cmd, mode=mode, queued=False,
                                            skipped='lens busy')
            queued = self._request_focus(f'{cmd} command', mode=mode, touch=touch)
            return self._result_payload(queued, action=cmd, mode=mode,
                                        queued=queued, touch=touch)

        if cmd == 'focus_infinity':
            queued = self._request_focus('focus_infinity command', mode='infinity')
            return self._result_payload(queued, action=cmd, queued=queued)

        # The mission-time lens setup, published by do_digi_cam_trigger when
        # DO_DIGICAM_CONTROL arrives. Queued onto the lens worker, and handled
        # ahead of the busy guard below, for the same two reasons focus is: it
        # must not run inline (the worker takes the camera lock itself), and
        # during a survey the pipeline is busy almost continuously at 1 Hz so
        # the guard would reject it for the whole flight.
        if cmd in {'capture_setup', 'zoom_capture'}:
            with_focus = cmd == 'capture_setup'
            queued = self._request_lens_setup(
                f'{cmd} command', zoom=True, focus=with_focus)
            return self._result_payload(queued, action=cmd, queued=queued,
                                        target_zoom_x=CAPTURE_ZOOM_X,
                                        focus=with_focus)

        if self.pipeline is not None and self.pipeline.is_busy():
            return self._result_payload(False, action=cmd, error='Pipeline is busy')

        if cmd == 'zoom_manual':
            direction_map = {'in': 1, 'out': -1, 'stop': 0}
            direction = param.lower()
            if direction not in direction_map:
                raise ValueError("zoom_manual parameter must be one of: in, out, stop")
            value = self.camera.manual_zoom(direction_map[direction])
            return self._result_payload(
                value is not None,
                action='zoom_manual',
                direction=direction,
                zoom_x=value,
            )

        if cmd in {'zoom_absolute', 'zoom_auto'}:
            if not param:
                raise ValueError(f"{cmd} requires a zoom multiple, e.g. 4.5")
            multiple = float(param)
            ok = self.camera.absolute_zoom_autofocus(multiple)
            if not self._capabilities()['zoom_readback']:
                self._set_zoom(multiple, 'commanded (this camera cannot report its zoom)')
            return self._result_payload(ok, action=cmd, target_zoom_x=multiple)

        if cmd == 'zoom_range':
            data = self.camera.get_supported_zoom_range()
            return self._result_payload(data is not None, action='zoom_range', data=data)

        if cmd == 'zoom_current':
            value = self.camera.get_current_zoom_magnification()
            self._set_zoom(value, 'readback')
            return self._result_payload(value is not None, action='zoom_current', zoom_x=value)

        if cmd == 'focus_manual':
            direction_map = {'far': 1, 'near': -1, 'stop': 0}
            direction = param.lower()
            if direction not in direction_map:
                raise ValueError("focus_manual parameter must be one of: far, near, stop")
            ok = self.camera.manual_focus(direction_map[direction])
            return self._result_payload(ok, action='focus_manual', direction=direction)

        if cmd == 'gimbal_rotate':
            yaw_raw, pitch_raw = self._split_csv(param, 2)
            yaw = int(yaw_raw)
            pitch = int(pitch_raw)
            ok = self.camera.rotate_gimbal(yaw_speed=yaw, pitch_speed=pitch)
            return self._result_payload(ok, action='gimbal_rotate', yaw=yaw, pitch=pitch)

        if cmd == 'gimbal_stop':
            ok = self.camera.stop_gimbal_rotation()
            return self._result_payload(ok, action='gimbal_stop')

        if cmd == 'gimbal_center':
            ok = self.camera.center_gimbal()
            return self._result_payload(ok, action='gimbal_center')

        if cmd == 'gimbal_attitude':
            data = self.camera.request_gimbal_attitude()
            return self._result_payload(data is not None, action='gimbal_attitude', data=data)

        if cmd == 'gimbal_set_angles':
            yaw_raw, pitch_raw = self._split_csv(param, 2)
            yaw = float(yaw_raw)
            pitch = float(pitch_raw)
            data = self.camera.set_gimbal_angles(yaw_deg=yaw, pitch_deg=pitch)
            return self._result_payload(
                data is not None,
                action='gimbal_set_angles',
                target_yaw_deg=yaw,
                target_pitch_deg=pitch,
                data=data,
            )

        if cmd == 'gimbal_set_axis':
            axis_raw, angle_raw = self._split_csv(param, 2)
            axis = axis_raw.lower()
            angle = float(angle_raw)
            data = self.camera.set_single_axis_angle(axis=axis, angle_deg=angle)
            return self._result_payload(
                data is not None,
                action='gimbal_set_axis',
                axis=axis,
                target_angle_deg=angle,
                data=data,
            )

        if cmd == 'gimbal_mode_get':
            mode = self.camera.get_gimbal_mode()
            return self._result_payload(mode is not None, action='gimbal_mode_get', mode=mode)

        if cmd == 'gimbal_mode_set':
            if not param:
                raise ValueError("gimbal_mode_set requires one of: lock, follow, fpv")
            mode = param.lower()
            ok = self.camera.set_gimbal_motion_mode(mode)
            mode_after = self.camera.get_gimbal_mode()
            return self._result_payload(
                ok,
                action='gimbal_mode_set',
                mode_requested=mode,
                mode_after=mode_after,
            )

        if cmd == 'sd_format':
            if param.lower() != 'yes':
                raise ValueError("sd_format is destructive; pass parameter yes to continue")
            ok = self.camera.format_sd_card()
            return self._result_payload(ok, action='sd_format')

        raise ValueError(f"Unsupported command: {command}")

    def camera_command_callback(self, msg: String):
        """Handle camera control commands from topic."""
        try:
            # Try parsing as JSON first
            try:
                data = json.loads(msg.data)
                command = data.get('command', '')
                parameter = data.get('parameter', '')
            except json.JSONDecodeError:
                # Fallback to simple format block: "command parameter"
                parts = msg.data.split(' ', 1)
                command = parts[0]
                parameter = parts[1] if len(parts) > 1 else ''
            
            if command.strip().lower().replace('-', '_') in QUEUED_COMMANDS:
                # Only queues work: no camera lock needed. (Taking the lock here
                # used to freeze this whole node -- GPS, height and photo
                # triggers included -- for as long as the lens worker held it.)
                result = self._execute_camera_command(command, parameter)
            elif self.camera_control_lock.acquire(timeout=CAMERA_COMMAND_LOCK_TIMEOUT_SECONDS):
                try:
                    result = self._execute_camera_command(command, parameter)
                finally:
                    self.camera_control_lock.release()
            else:
                result = self._result_payload(
                    False, error=f'camera busy for {CAMERA_COMMAND_LOCK_TIMEOUT_SECONDS:.0f} s '
                                 '(a photo or a lens move) - send the command again')

            success = bool(result.get('ok', False))
            
            if success:
                if result.get('skipped'):
                    self._publish_camera_status(f"CMD SKIPPED: {command} - {result['skipped']}")
                else:
                    self._publish_camera_status(f"CMD SUCCESS: {command}")
                # self.get_logger().info(f"Camera command success: {command}")
            else:
                self._publish_camera_status(f"CMD FAILED: {command} - {result.get('error', 'Unknown')}")
                self.get_logger().error(f"Camera command failed: {command} - {result.get('error', 'Unknown')}")

        except (ValueError, CameraConnectionError, OSError) as exc:
            self._publish_camera_status(f"CMD ERROR: {msg.data}")
            self.get_logger().error(f"Camera command error for {msg.data}: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected camera command error for {msg.data}: {exc}")
            self._publish_camera_status(f"CMD ERROR: {msg.data}")
    
    def _execute_simulation_capture(self):
        """Execute simulation capture (save current image)"""
        if self.latest_image_msg is None:
            self.get_logger().warn("Trigger received but no simulation image available")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')

            # Rotate image to fix upside-down physical mounting
            if self.rotate_180:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            
            # Save to mapping directory
            mapping_dir = self.storage.get_mapping_dir()
            filename = self._generate_gps_filename()
            if filename is None:
                filename = f"sim_{time.strftime('%Y%m%d-%H%M%S')}.jpg"
            filepath = os.path.join(mapping_dir, filename)
            
            cv2.imwrite(filepath, cv_image)
            self._photos_saved += 1
            self.get_logger().info(f"Simulation photo saved: {filepath}")
            self._send_status(f"Simulation photo captured: {time.strftime('%Y%m%d-%H%M%S')}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save simulation image: {e}")

    def _generate_gps_filename(self) -> Optional[str]:
        """Build a '<lat> , <lon>.jpg' filename from the latest cached fix.

        IMPORTANT: the '" , "' delimiter (space-comma-space) is parsed by
        detection/new_od.py on the consuming side via `name.split(' , ')`,
        which then does `float()` on each half to populate
        ImageResult.latitude / ImageResult.longitude. Do NOT change the
        delimiter or append anything else to the stem without updating
        new_od's parser in lockstep, or main_controller will start placing
        detection waypoints at (0.0, 0.0).
        """
        if self.latest_gps is None:
            return None

        try:
            lat = float(self.latest_gps.latitude)
            lon = float(self.latest_gps.longitude)
            self.get_logger().info(f"{self.latest_gps.latitude}, {self.latest_gps.longitude}")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"Cached NavSatFix has invalid coordinates: {exc}")
            return None

        # Null Island: the FCU publishes (0.0, 0.0) before GPS lock, so
        # treat it as "no fix" rather than baking it into a filename.
        if lat == 0.0 and lon == 0.0:
            self.get_logger().warn(
                "Cached GPS fix is (0.0, 0.0) - treating as no fix")
            return None

        return f"{lat:.6f} , {lon:.6f}.jpg"

    def _publish_captured_image(self, img: np.ndarray, saved_path: str):
        """Publish an already-decoded image to ROS /image_raw.

        The pipeline's download_and_save() hands back the decoded ndarray
        directly, so this method publishes it without a round-trip through
        cv2.imread(). Saves one full-resolution disk read per capture.

        Args:
            img: Decoded BGR image (as returned by download_and_save).
            saved_path: Absolute path the pipeline wrote — used only for
                the log line and the ROS header's frame_id context.
        """
        try:
            if img is None:
                self.get_logger().warn(
                    "Pipeline handed back a None image; "
                    "nothing to publish on /image_raw")
                return

            msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.image_pub.publish(msg)
            # self.get_logger().info(
            #     f"Published image to image_raw: {os.path.basename(saved_path)}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish captured image: {e}")
    
    def _publish_disk_status(self):
        """Publish disk space status"""
        free_mb = self.storage.get_free_space_mb()
        msg = Float64()
        msg.data = free_mb
        self.disk_status_pub.publish(msg)
    
    def camera_trigger_callback(self, msg: Bool):
        """Handle capture trigger requests: every one is queued, none is thrown away."""
        if msg.data:
            self.get_logger().info("Capture trigger received!")
            if not self.capture_queue.push(time.time()):
                self.get_logger().warn(
                    f"Photo queue full ({MAX_PENDING_CAPTURES} waiting): dropped the OLDEST "
                    f"request ({self.capture_queue.dropped} dropped so far) - the camera cannot "
                    f"keep up")
            self._service_capture_queue()
        else:
            self.get_logger().debug("Trigger received with data=False, ignoring")
    
    def altitude_callback(self, msg: Float64):
        """Handle altitude updates for camera enable/disable"""
        current_alt = msg.data
        self.latest_rel_alt = current_alt
        self.latest_rel_alt_time = time.time()
        was_enabled = self.camera_enabled
        self._check_climb_focus(current_alt)

        if current_alt >= self.altitude_threshold:
            self.camera_enabled = True
            if not was_enabled:
                self.get_logger().info(
                    f"Altitude {current_alt:.2f}m >= {self.altitude_threshold:.2f}m - "
                    "Camera ENABLED")
                self._send_status("Altitude threshold reached - Camera enabled")
        else:
            self.camera_enabled = False
            if was_enabled:
                self.get_logger().info(
                    f"Altitude {current_alt:.2f}m < {self.altitude_threshold:.2f}m - "
                    "Camera DISABLED")
                self._send_status("Below altitude threshold - Camera disabled")
    
    def sim_image_callback(self, msg: Image):
        """Callback for simulation images"""
        self.latest_image_msg = msg
    
    def _send_status(self, text: str):
        """Send status message to MAVROS"""
        msg = StatusText()
        msg.severity = 6
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def _publish_camera_status(self, text: str):
        """Publish camera-specific status"""
        msg = String()
        msg.data = text
        self.camera_status_pub.publish(msg)
    
    def shutdown(self):
        """Proper shutdown handler"""
        # try:
        #     self.get_logger().info("Shutting down SIYI pipeline...")
        # except Exception:
        #     pass
        try:
            self._log_capture_stats(force=True)
        except Exception:
            pass
        
        # Close camera interface
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass
        
        try:
            self.get_logger().info("Shutdown complete")
        except Exception:
            pass


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SIYINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Initiating shutdown...")
        node.shutdown()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
