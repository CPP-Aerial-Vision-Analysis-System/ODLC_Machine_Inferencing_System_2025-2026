"""
sim_demo/camera_model.py -- PURPOSE: the camera math of the simulated SIYI camera: where a
point on the ground shows up in a photo (pixel), which patch of ground a photo covers (its
"footprint"), and where a target's box lands in the picture.

Used by truth_detector (to decide which targets a photo shows and draw their boxes), by
demo_markers (to draw photo footprints in RViz) and by score_run (to know which targets
each photo could have shown).

Conventions (all angles in degrees, all distances in metres):
  * World frame = Gazebo world ENU: x east, y north, z up (origin = SITL home).
  * Camera pose: position (east, north, up) and three angles:
      yaw_deg   = compass heading the TOP of the image points to (0 = north, 90 = east).
                  With the gimbal looking straight down and its yaw centred, this equals
                  the drone's heading (the image top points out of the drone's nose).
      pitch_deg = gimbal tilt: 0 = looking at the horizon, -90 = straight down (SIYI sign).
      roll_deg  = rotation of the picture about the optical axis (normally 0: stabilised).
  * Pixels: OpenCV convention, (0, 0) = top-left corner, u to the right, v down.
  * Pinhole model with square pixels: fx = fy = (width / 2) / tan(effective_hfov / 2).
    Zoom (digital crop or optical) narrows the view: tan(hfov_zoomed/2) = tan(hfov/2) / zoom.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# Horizontal field of view of the simulated camera, from the contract (A8 mini: 81 deg).
DEFAULT_HFOV_RAD = 1.4137

# Photo resolutions of the two camera profiles in the contract.
CAMERA_PROFILES = {
    'zr10': (2560, 1440),     # DEFAULT: last year's code targets the ZR10 2K profile
    'a8mini': (3840, 2160),   # A8 mini 4K stills
    'video': (1280, 720),     # the live video camera
}

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Size and field of view of a camera image.

    Attributes:
        width, height: image size in pixels.
        hfov_rad: horizontal field of view at zoom 1.0, radians.
        zoom: zoom factor (1.0 = widest). 5.0 = the ZR10 capture zoom last year's code uses.
    """

    width: int
    height: int
    hfov_rad: float = DEFAULT_HFOV_RAD
    zoom: float = 1.0

    @classmethod
    def from_profile(cls, profile: str, zoom: float = 1.0,
                     hfov_rad: float = DEFAULT_HFOV_RAD) -> 'CameraIntrinsics':
        """Build intrinsics for a named profile ('zr10', 'a8mini' or 'video').

        Raises:
            KeyError: unknown profile name.
        """
        width, height = CAMERA_PROFILES[profile.lower()]
        return cls(width, height, hfov_rad, zoom)

    @property
    def effective_hfov_rad(self) -> float:
        """Horizontal field of view after zoom, radians."""
        z = max(1e-6, float(self.zoom))
        return 2.0 * math.atan(math.tan(self.hfov_rad / 2.0) / z)

    @property
    def fx(self) -> float:
        """Focal length in pixels (same for x and y: square pixels)."""
        return (self.width / 2.0) / math.tan(self.effective_hfov_rad / 2.0)

    @property
    def fy(self) -> float:
        """Focal length in pixels along y (= fx)."""
        return self.fx

    @property
    def cx(self) -> float:
        """Image centre x in pixels."""
        return self.width / 2.0

    @property
    def cy(self) -> float:
        """Image centre y in pixels."""
        return self.height / 2.0

    @property
    def vfov_rad(self) -> float:
        """Vertical field of view after zoom, radians."""
        return 2.0 * math.atan((self.height / 2.0) / self.fy)


@dataclass(frozen=True)
class CameraPose:
    """Where the camera is and where it looks (see the module docstring for conventions).

    `axes` (optional) gives the camera frame directly as (x_right, y_down, z_forward) unit
    vectors in world ENU - e.g. from the camera emulator's optical-frame quaternion - and then
    overrides yaw/pitch/roll (which are ambiguous when looking straight down).
    """

    east: float
    north: float
    up: float
    yaw_deg: float = 0.0
    pitch_deg: float = -90.0
    roll_deg: float = 0.0
    axes: tuple | None = None


def axes_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[Vec3, Vec3, Vec3]:
    """Camera axes from the (x, y, z, w) quaternion of an OPTICAL frame (x right, y down,
    z forward), as published by the camera emulator ("optical_quat_xyzw").

    Returns:
        (x_right, y_down, z_forward) unit vectors in world ENU = the rotation matrix columns.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    col0 = (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w))
    col1 = (2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w))
    col2 = (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y))
    return col0, col1, col2


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def camera_axes(pose: CameraPose) -> tuple[Vec3, Vec3, Vec3]:
    """Unit vectors of the camera frame expressed in world ENU.

    Returns:
        (x_right, y_down, z_forward): image-right, image-down and the optical axis (the
        direction the camera looks), each a unit (east, north, up) vector.
    """
    if pose.axes is not None:
        return pose.axes
    psi = math.radians(pose.yaw_deg)      # compass heading of the image top
    theta = math.radians(pose.pitch_deg)  # tilt, -90 = down
    # Optical axis: tilt theta from the horizontal, in the vertical plane of heading psi.
    z_fwd = (math.cos(theta) * math.sin(psi), math.cos(theta) * math.cos(psi), math.sin(theta))
    # "Image up" lies in the same vertical plane, 90 degrees above the optical axis.
    up_img = (-math.sin(theta) * math.sin(psi), -math.sin(theta) * math.cos(psi), math.cos(theta))
    y_down = (-up_img[0], -up_img[1], -up_img[2])
    x_right = _cross(y_down, z_fwd)
    if abs(pose.roll_deg) > 1e-9:
        # Roll the picture about the optical axis (positive = image rotates clockwise as seen
        # by the camera, i.e. the right side of the picture goes down).
        r = math.radians(pose.roll_deg)
        c, s = math.cos(r), math.sin(r)
        x_new = tuple(c * x_right[i] + s * y_down[i] for i in range(3))
        y_new = tuple(-s * x_right[i] + c * y_down[i] for i in range(3))
        x_right, y_down = x_new, y_new  # type: ignore[assignment]
    return x_right, y_down, z_fwd


def project_point(intr: CameraIntrinsics, pose: CameraPose,
                  point: Vec3) -> tuple[float, float] | None:
    """Pixel where a world point appears (it may be outside the image).

    Args:
        intr: camera intrinsics. pose: camera pose. point: (east, north, up) metres.
    Returns:
        (u, v) pixels, or None if the point is behind the camera.
    """
    x_r, y_d, z_f = camera_axes(pose)
    d = (point[0] - pose.east, point[1] - pose.north, point[2] - pose.up)
    zc = _dot(d, z_f)
    if zc <= 1e-6:
        return None
    u = intr.fx * _dot(d, x_r) / zc + intr.cx
    v = intr.fy * _dot(d, y_d) / zc + intr.cy
    return u, v


def pixel_ray(intr: CameraIntrinsics, pose: CameraPose, u: float, v: float) -> Vec3:
    """World direction (not normalised) of the ray through pixel (u, v)."""
    x_r, y_d, z_f = camera_axes(pose)
    a = (u - intr.cx) / intr.fx
    b = (v - intr.cy) / intr.fy
    return tuple(x_r[i] * a + y_d[i] * b + z_f[i] for i in range(3))  # type: ignore[return-value]


def pixel_to_ground(intr: CameraIntrinsics, pose: CameraPose, u: float, v: float,
                    ground_up: float = 0.0, max_range: float = 2000.0) -> tuple[float, float] | None:
    """Ground point (east, north) seen at pixel (u, v), assuming flat ground at height ground_up.

    Returns:
        (east, north) metres, or None if that pixel looks at the sky or farther than max_range.
    """
    r = pixel_ray(intr, pose, u, v)
    if r[2] >= -1e-9:
        return None
    t = (ground_up - pose.up) / r[2]
    if t <= 0:
        return None
    e = pose.east + t * r[0]
    n = pose.north + t * r[1]
    if math.hypot(e - pose.east, n - pose.north) > max_range:
        return None
    return e, n


def footprint(intr: CameraIntrinsics, pose: CameraPose, ground_up: float = 0.0,
              max_range: float = 2000.0) -> list[tuple[float, float]] | None:
    """The patch of flat ground a photo covers, as 4 corners (east, north).

    Order: top-left, top-right, bottom-right, bottom-left of the image.

    Returns:
        list of 4 (east, north) points, or None if a corner looks above the horizon
        (an oblique photo whose footprint is unbounded).
    """
    corners = [(0.0, 0.0), (intr.width, 0.0), (intr.width, intr.height), (0.0, intr.height)]
    pts = []
    for u, v in corners:
        g = pixel_to_ground(intr, pose, u, v, ground_up, max_range)
        if g is None:
            return None
        pts.append(g)
    return pts


def point_in_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    """True if (x, y) is inside the polygon (ray casting; edges count as inside-ish)."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def polygon_area(poly: list[tuple[float, float]]) -> float:
    """Area of a simple polygon (shoelace formula), square metres or square pixels."""
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def ground_sample_distance(intr: CameraIntrinsics, height_above_ground: float) -> float:
    """Metres of ground per pixel at the image centre for a straight-down photo."""
    return height_above_ground / intr.fx


def box_corners_3d(east: float, north: float, up: float, length: float, width: float,
                   height: float, yaw_deg: float = 0.0) -> list[Vec3]:
    """The 8 corners of an object's box.

    Args:
        east, north, up: centre of the box BOTTOM (where it touches the ground), metres.
        length: size along the object's own x axis; width: along its y axis; height: up.
        yaw_deg: ENU yaw of the object's x axis (0 = east, counter-clockwise).
    Returns:
        list of 8 (east, north, up) points.
    """
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    pts = []
    for dx in (-length / 2.0, length / 2.0):
        for dy in (-width / 2.0, width / 2.0):
            for dz in (0.0, height):
                pts.append((east + c * dx - s * dy, north + s * dx + c * dy, up + dz))
    return pts


@dataclass(frozen=True)
class ProjectedBox:
    """Where an object's box falls in an image.

    Attributes:
        x1, y1, x2, y2: the part of the box INSIDE the image, pixels (x1 < x2, y1 < y2).
        visible_fraction: 0..1, how much of the full (unclipped) box area is inside the image.
        full_width, full_height: unclipped box size in pixels.
        center_norm: distance of the visible box centre from the image centre, divided by the
            half-diagonal (0 = centre, 1 = a corner).
    """

    x1: float
    y1: float
    x2: float
    y2: float
    visible_fraction: float
    full_width: float
    full_height: float
    center_norm: float

    @property
    def cx(self) -> float:
        """Centre x of the visible box, pixels."""
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        """Centre y of the visible box, pixels."""
        return (self.y1 + self.y2) / 2.0

    @property
    def width(self) -> float:
        """Visible box width, pixels."""
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        """Visible box height, pixels."""
        return self.y2 - self.y1

    @property
    def longest_side_px(self) -> float:
        """Longest side of the FULL box in pixels (how big the object looks)."""
        return max(self.full_width, self.full_height)


def project_box(intr: CameraIntrinsics, pose: CameraPose, corners: list[Vec3]) -> ProjectedBox | None:
    """Project an object's 3D box corners into the image and clip to the image.

    Returns:
        ProjectedBox, or None if any corner is behind the camera or nothing is inside the image.
    """
    uv = []
    for p in corners:
        q = project_point(intr, pose, p)
        if q is None:
            return None
        uv.append(q)
    us = [q[0] for q in uv]
    vs = [q[1] for q in uv]
    fx1, fx2, fy1, fy2 = min(us), max(us), min(vs), max(vs)
    full_w, full_h = fx2 - fx1, fy2 - fy1
    x1, x2 = max(0.0, fx1), min(float(intr.width), fx2)
    y1, y2 = max(0.0, fy1), min(float(intr.height), fy2)
    if x2 <= x1 or y2 <= y1:
        return None
    full_area = max(1e-9, full_w * full_h)
    visible = ((x2 - x1) * (y2 - y1)) / full_area
    half_diag = math.hypot(intr.width / 2.0, intr.height / 2.0)
    cn = math.hypot((x1 + x2) / 2.0 - intr.cx, (y1 + y2) / 2.0 - intr.cy) / half_diag
    return ProjectedBox(x1, y1, x2, y2, min(1.0, visible), full_w, full_h, cn)
