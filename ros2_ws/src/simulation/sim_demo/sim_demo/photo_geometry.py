"""
sim_demo/photo_geometry.py -- PURPOSE: for each new photo file, work out the camera's pose and
zoom at the moment it was taken, and the patch of ground it shows (its footprint). The
truth_detector stand-in, the RViz markers and the scorer all use this one piece of logic, so
they always agree with each other.

Where the numbers come from, best first:
  1. the camera emulator's own record of the photo (/sim/camera/photo_info, see
     photo_info.py): exact camera position and orientation from Gazebo, zoom, field of view;
  2. otherwise: photo size from the JPEG header, drone position / height / heading at
     shutter time from the recorded MAVROS history matched to the GPS in the file name
     (pose_history.py), the camera zoom from zoom.py, and the gimbal straight down unless
     configured otherwise.
Pure Python (no ROS imports): `flight` is any object with capture_pose(...) and a `zoom`
attribute that has zoom_at(t) (ros_common.FlightData in the nodes, a fake in tests).
"""
from __future__ import annotations

from dataclasses import dataclass

from .camera_model import DEFAULT_HFOV_RAD, CameraIntrinsics, CameraPose, footprint
from .geo import GeoOrigin
from .photo_info import PhotoInfoIndex
from .photos import PhotoEvent, jpeg_size
from .pose_history import PoseSample
from .truth_model import pose_from_sample


@dataclass
class PhotoSolution:
    """Everything known about one photo's geometry.

    Attributes:
        event: the photo file event. intr: intrinsics (size, field of view, zoom).
        pose: camera pose in world ENU (None if the drone state at capture is unknown).
        sample: the drone state from the MAVROS history (None if not used/unknown).
        how: 'emulator' (exact record), 'gps-match' / 'time' (MAVROS history) or 'none'.
        zoom_source: where the zoom value came from.
        footprint: 4 (east, north) ground corners, or None.
    """

    event: PhotoEvent
    intr: CameraIntrinsics
    pose: CameraPose | None
    sample: PoseSample | None
    how: str
    zoom_source: str
    footprint: list | None


class PhotoGeometry:
    """Solves photo poses and footprints (see the module docstring)."""

    def __init__(self, origin: GeoOrigin, flight, hfov_rad: float = DEFAULT_HFOV_RAD,
                 gimbal_pitch_deg: float = -90.0, gimbal_yaw_offset_deg: float = 0.0,
                 capture_latency_s: float = 1.0, default_size: tuple[int, int] = (2560, 1440),
                 photo_info: PhotoInfoIndex | None = None):
        """
        Args:
            origin: world GPS origin. flight: provider of capture_pose() and zoom.zoom_at().
            hfov_rad: camera horizontal field of view at zoom 1 [rad].
            gimbal_pitch_deg: camera tilt during the search (-90 = straight down).
            gimbal_yaw_offset_deg: camera yaw relative to the nose (+ = right).
            capture_latency_s: typical shutter-to-file delay (used only without GPS in the name).
            default_size: photo size if the file header cannot be read.
            photo_info: the camera emulator's photo records (None = not used).
        """
        self.origin = origin
        self.flight = flight
        self.hfov_rad = hfov_rad
        self.gimbal_pitch_deg = gimbal_pitch_deg
        self.gimbal_yaw_offset_deg = gimbal_yaw_offset_deg
        self.capture_latency_s = capture_latency_s
        self.default_size = default_size
        self.photo_info = photo_info

    def solve(self, event: PhotoEvent) -> PhotoSolution:
        """Work out the geometry of one photo (never raises; unknown parts are None)."""
        sample, how = (None, 'none')
        if not event.preexisting:
            # When was the drone at the file name's GPS? (= when video_cam chose the name)
            sample, how = self.flight.capture_pose(self.origin, event.lat, event.lon, event.mtime,
                                                   latency_s=self.capture_latency_s)
        if self.photo_info is not None and not event.preexisting:
            name_time = sample.t if (sample is not None and how == 'gps-match') else None
            rec = self.photo_info.match(event.mtime, event.lat, event.lon, name_time)
            if rec is not None:
                return PhotoSolution(event, rec.intr, rec.pose, None, 'emulator',
                                     'camera emulator', footprint(rec.intr, rec.pose))
        size = jpeg_size(event.path) or self.default_size
        t_capture = sample.t if sample is not None else event.mtime
        zoom, zsrc = self.flight.zoom.zoom_at(t_capture)
        intr = CameraIntrinsics(int(size[0]), int(size[1]), self.hfov_rad, zoom)
        if sample is None:
            return PhotoSolution(event, intr, None, None, how, zsrc, None)
        pose = pose_from_sample(self.origin, sample, self.gimbal_pitch_deg,
                                self.gimbal_yaw_offset_deg)
        return PhotoSolution(event, intr, pose, sample, how, zsrc, footprint(intr, pose))
