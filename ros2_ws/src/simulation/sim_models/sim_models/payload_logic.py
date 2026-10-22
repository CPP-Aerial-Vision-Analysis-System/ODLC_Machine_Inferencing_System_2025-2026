"""
sim_models/payload_logic.py -- PURPOSE: the decisions behind the simulated payload drop, written as
plain Python classes (no ROS, no Gazebo) so they can be unit tested on any computer.

  ServoReleaseDetector  "Should this payload fall now?"  It watches the pulse width (microseconds)
                        of the payload's release servo, as ArduPilot SITL outputs it, and says
                        YES exactly once: the first time the servo is OPEN (>= the open threshold).
  LandingDetector       "Has the falling payload landed?"  It watches the payload's world position
                        after the release and says YES once it has stopped moving.
  enu_to_geodetic       East/North/Up metres in the Gazebo world -> latitude/longitude/altitude,
                        so a landing point can be compared with GPS positions.

Why "first reading at or above the threshold" (and not "a jump from closed to open")?
  The team's flight code (main_controller.py) opens a drop servo with MAV_CMD_DO_SET_SERVO to
  1400 us, waits 3 s and closes it with 500 us. Measured with ArduCopter 4.6.3 SITL + Gazebo:
  before the first DO_SET_SERVO the simulator link carries SITL's start value 1000 us for these
  outputs (SERVO_OUTPUT_RAW reports 0), and ArduPilotPlugin publishes NOTHING while a value is
  exactly 0 (before SITL connects). A release node started late may therefore see 1400 = open as
  its very first value, and must still drop. So: no reading, 0 us, 500 us, 1000 us and anything
  else below the threshold mean "closed"; the first value at or above it means "open".
"""

import math

# The flight code's servo positions (main_controller.py:28-29, "bench-verified 500-1400 travel").
OPEN_PWM_US = 1400.0
CLOSED_PWM_US = 500.0

# The relayed value is 3000 * (pwm / 3000) in floating point, so 1400 may arrive as 1399.9999999.
# Half a microsecond of tolerance absorbs that without changing the meaning of the threshold.
PWM_TOLERANCE_US = 0.5


class ServoReleaseDetector:
    """Decide when ONE payload must be released, from its release servo's pulse width.

    Usage:
        det = ServoReleaseDetector(open_threshold_us=1400.0)
        if det.update(pwm_us=1400.0, t=12.3):
            ...detach the payload...      # True only once

    Attributes (read only):
        released          True after the release decision was made.
        release_time      time [s] passed to update() when it released (None before).
        release_pwm_us    the pulse width [us] that caused the release (None before).
        last_pwm_us       the latest pulse width seen [us] (None = nothing received yet).
    """

    def __init__(self, open_threshold_us: float = OPEN_PWM_US, min_open_time_s: float = 0.0):
        """
        Args:
            open_threshold_us: pulse width [us] at or above which the servo counts as OPEN.
                The flight code opens with 1400 us, so the default is 1400.
            min_open_time_s: how long [s] the servo must stay open before releasing.
                0 (default) = release on the first open reading. Use e.g. 0.2 to ignore glitches.

        Raises:
            ValueError: if open_threshold_us is not positive or min_open_time_s is negative.
        """
        if not open_threshold_us > 0.0:
            raise ValueError(f"open_threshold_us must be > 0 (got {open_threshold_us})")
        if min_open_time_s < 0.0:
            raise ValueError(f"min_open_time_s must be >= 0 (got {min_open_time_s})")
        self.open_threshold_us = float(open_threshold_us)
        self.min_open_time_s = float(min_open_time_s)
        self.released = False
        self.release_time = None
        self.release_pwm_us = None
        self.last_pwm_us = None
        self._open_since = None

    def is_open(self, pwm_us: float) -> bool:
        """Return True if this pulse width [us] means OPEN (0, negative or NaN mean closed)."""
        if pwm_us is None or math.isnan(pwm_us):
            return False
        return pwm_us >= self.open_threshold_us - PWM_TOLERANCE_US

    def update(self, pwm_us: float, t: float) -> bool:
        """Feed one servo reading.

        Args:
            pwm_us: pulse width in microseconds (as relayed from ArduPilot SITL).
            t: time of the reading in seconds (any clock, it only has to increase).

        Returns:
            True exactly once: on the reading that decides the release. False otherwise
            (still closed, not open long enough yet, or already released).
        """
        self.last_pwm_us = pwm_us
        if self.released:
            return False
        if not self.is_open(pwm_us):
            self._open_since = None
            return False
        if self._open_since is None:
            self._open_since = t
        if t - self._open_since + 1e-9 < self.min_open_time_s:
            return False
        self.released = True
        self.release_time = t
        self.release_pwm_us = pwm_us
        return True


class LandingDetector:
    """Decide when a dropped payload has come to rest, from its world positions.

    Usage:
        land = LandingDetector()
        land.start(t, x, y, z)            # at the release
        if land.update(t, x, y, z): ...   # True once, when it has landed

    Attributes (read only):
        start_point   (t, x, y, z) at the release, or None.
        landed_point  (t, x, y, z) where it came to rest, or None.
    """

    def __init__(
        self, rest_speed: float = 0.05, rest_time_s: float = 1.0, min_fall_time_s: float = 0.3
    ):
        """
        Args:
            rest_speed: speed [m/s] below which the payload counts as not moving.
            rest_time_s: it must stay below rest_speed this long [s] to count as landed.
            min_fall_time_s: ignore the first part of the fall [s] (the payload starts at rest
                relative to a hovering drone, so it would otherwise look "landed" at once).
        """
        if rest_speed <= 0.0 or rest_time_s < 0.0 or min_fall_time_s < 0.0:
            raise ValueError("rest_speed must be > 0, rest_time_s and min_fall_time_s >= 0")
        self.rest_speed = float(rest_speed)
        self.rest_time_s = float(rest_time_s)
        self.min_fall_time_s = float(min_fall_time_s)
        self.start_point = None
        self.landed_point = None
        self._last = None
        self._rest_since = None

    @property
    def started(self) -> bool:
        """True once start() was called."""
        return self.start_point is not None

    @property
    def landed(self) -> bool:
        """True once the payload has come to rest."""
        return self.landed_point is not None

    def start(self, t: float, x: float, y: float, z: float) -> None:
        """Remember the release time and position (world metres)."""
        self.start_point = (t, x, y, z)
        self._last = (t, x, y, z)
        self._rest_since = None
        self.landed_point = None

    def update(self, t: float, x: float, y: float, z: float) -> bool:
        """Feed one world position sample (seconds, metres).

        Returns:
            True exactly once, when the payload has been at rest for rest_time_s.
            False before start(), while it still moves, and after it has landed.
        """
        if self.start_point is None or self.landed_point is not None:
            return False
        last = self._last
        self._last = (t, x, y, z)
        dt = t - last[0]
        if dt <= 1e-6:
            return False
        speed = math.dist((x, y, z), last[1:]) / dt
        if t - self.start_point[0] < self.min_fall_time_s or speed >= self.rest_speed:
            self._rest_since = None
            return False
        if self._rest_since is None:
            self._rest_since = last[0]
        if t - self._rest_since + 1e-9 >= self.rest_time_s:
            self.landed_point = (t, x, y, z)
            return True
        return False

    def fall_summary(self) -> dict:
        """Return {'fall_time_s', 'fall_height_m', 'drift_m'} once landed (empty dict before)."""
        if self.start_point is None or self.landed_point is None:
            return {}
        t0, x0, y0, z0 = self.start_point
        t1, x1, y1, z1 = self.landed_point
        return {
            "fall_time_s": round(t1 - t0, 3),
            "fall_height_m": round(z0 - z1, 3),
            "drift_m": round(math.hypot(x1 - x0, y1 - y0), 3),
        }


# WGS-84 ellipsoid (the same one Gazebo's <spherical_coordinates> EARTH_WGS84 uses)
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def enu_to_geodetic(
    east: float, north: float, up: float, lat0_deg: float, lon0_deg: float, alt0_m: float = 0.0
):
    """Convert local East/North/Up metres to latitude, longitude [deg] and altitude [m].

    The local frame is the Gazebo world frame of a world whose <spherical_coordinates> has
    latitude lat0_deg, longitude lon0_deg, elevation alt0_m and heading 0 (x = East, y = North).
    It uses the local radii of curvature of the WGS-84 ellipsoid, which is accurate to a few
    millimetres over the size of a SUAS field (well under 10 km).

    Args:
        east, north, up: position in metres (Gazebo x, y, z).
        lat0_deg, lon0_deg, alt0_m: the world origin (spherical_coordinates).

    Returns:
        (latitude_deg, longitude_deg, altitude_m)
    """
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    w = math.sqrt(1.0 - _WGS84_E2 * s * s)
    r_north = _WGS84_A * (1.0 - _WGS84_E2) / (w**3)  # meridian radius of curvature
    r_east = _WGS84_A / w  # prime vertical radius of curvature
    lat = lat0_deg + math.degrees(north / r_north)
    lon = lon0_deg + math.degrees(east / (r_east * math.cos(lat0)))
    return lat, lon, alt0_m + up
