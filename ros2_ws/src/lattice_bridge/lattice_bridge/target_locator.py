"""
lattice_bridge/target_locator.py -- PURPOSE: work out WHERE ON THE GROUND a detected hiker or tent is, using the
team's own code, so the position the reporter sends to the ground is the position main_controller will aim at.

main_controller turns "the detector saw a person at pixel (u, v) of photo X" into latitude / longitude in four
steps (ros2_ws/src/main/src/main_controller.py, locate_detection): read the photo's JSON sidecar, take the
drone position from the sidecar (or the photo's file name), check the box centre, and project the box centre onto
the ground with main/src/target_geolocation.py (aim_at_box). This module does the SAME steps and calls the SAME
function. It does NOT copy the maths: it finds the installed file of package `main`
(<install>/main/lib/main/target_geolocation.py, found through AMENT_PREFIX_PATH exactly like ament_index does;
or, in a source checkout, <repo>/ros2_ws/src/main/src/target_geolocation.py) and loads it. If the file cannot
be found or has changed shape, TeamModuleError says so in plain words and the reporter does not start; the
flight is not affected either way. Parity with main_controller is tested against its real
`locate_detection` method (test_target_locator.py).

What it adds on top of main_controller, on purpose:
  * a detection whose position fell back to the DRONE'S own position (no sidecar, no height ...) is skipped
    unless `allow_photo_fallback` is set: Lattice must show where the target is, never where the drone was;
  * every problem comes back as a short key and a sentence (LocateResult.skip_key / skip_text) instead of a
    log line, so the node can count it and log each kind once;
  * the photo name must be a plain file name (no folders, no NUL), because the name arrives in a ROS message.

Also here: extract_job() copies the few numbers needed out of an interfaces/ImageResult message into plain
Python (so no ROS object waits in a queue), and default_photo_dir() finds the photo folder the way
main_controller does.

Python standard library only. It reads files (the sidecar and the JPEG header) through the team's functions;
the node calls it from a timer, never from the message callback.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
from dataclasses import dataclass
from typing import Any, NamedTuple, Optional, Tuple

from . import constants as C
from .reporter_logic import AimSettings

TEAM_PACKAGE = 'main'
TEAM_MODULE_FILE = 'target_geolocation.py'
_PRIVATE_MODULE_NAME = 'astra_team_target_geolocation'
#: Everything this module calls in the team's file. A missing name means the team code changed shape.
REQUIRED_NAMES = ('aim_at_box', 'photo_position', 'read_photo_sidecar', 'jpeg_size', 'sidecar_path_for',
                  'is_finite_number', 'valid_latlon', 'AimResult')

#: class_id "0" is a person, "1" a tent: the same table as main_controller.image_result_cb.
KIND_BY_CLASS_ID = {'0': C.KIND_PERSON, '1': C.KIND_TENT}
#: One answer of the detector is never trusted for more than this many boxes (bounded work).
MAX_DETECTIONS_PER_PHOTO = 64
MAX_NAME_LENGTH = 255

#: Why a detection is not reported (LocateResult.skip_key).
SKIP_BAD_NAME = 'bad_name'
SKIP_NO_POSITION = 'no_position'
SKIP_BAD_BOX = 'bad_box'
SKIP_NO_AIM = 'no_aim'
SKIP_PHOTO_FALLBACK = 'photo_fallback'

_loaded_modules: dict = {}


class TeamModuleError(RuntimeError):
    """The team's target_geolocation.py could not be found, loaded or understood."""


# ---------------------------------------------------------------------------------------------------
# Finding and loading the team's own projection code
# ---------------------------------------------------------------------------------------------------

def find_team_module(environ=None, search_from: Optional[str] = None) -> Tuple[Optional[str], str]:
    """Return (path, how it was found) of the team's target_geolocation.py, or (None, why not).

    1. The installed package `main`: every folder of AMENT_PREFIX_PATH that holds the package marker
       share/ament_index/resource_index/packages/main and the file lib/main/target_geolocation.py (the same
       lookup ament_index_python.get_package_prefix does, without importing it).
    2. A source checkout: walking up from this file (or `search_from`) to a folder that holds
       main/src/target_geolocation.py (that folder is ros2_ws/src). Used by tests and by symlink-style dev runs.
    """
    env = os.environ if environ is None else environ
    for prefix in (env.get('AMENT_PREFIX_PATH') or '').split(os.pathsep):
        if not prefix:
            continue
        marker = os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages', TEAM_PACKAGE)
        candidate = os.path.join(prefix, 'lib', TEAM_PACKAGE, TEAM_MODULE_FILE)
        if os.path.isfile(marker) and os.path.isfile(candidate):
            return candidate, f'installed package {TEAM_PACKAGE} ({prefix})'
    here = os.path.dirname(os.path.abspath(search_from or __file__))
    for _ in range(8):
        candidate = os.path.join(here, TEAM_PACKAGE, 'src', TEAM_MODULE_FILE)
        if os.path.isfile(candidate):
            return candidate, f'source tree ({here})'
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return None, (f'package {TEAM_PACKAGE} is not installed (source the workspace: '
                  f'source ros2_ws/install/setup.bash) and no source checkout was found next to this package')


def load_team_module(path: str):
    """Load the team's target_geolocation.py from `path` under a private name and check it has what we call.

    Raises TeamModuleError (with a sentence for a student) if the file cannot be loaded or lacks a name in
    REQUIRED_NAMES. The result is cached per path.
    """
    if path in _loaded_modules:
        return _loaded_modules[path]
    private_name = f'{_PRIVATE_MODULE_NAME}_{len(_loaded_modules)}_{abs(hash(path)) % 100000}'
    spec = importlib.util.spec_from_file_location(private_name, path)
    if spec is None or spec.loader is None:
        raise TeamModuleError(f'cannot load {path}: it is not a Python file')
    module = importlib.util.module_from_spec(spec)
    sys.modules[private_name] = module                   # a dataclass inside the file looks itself up here
    try:
        spec.loader.exec_module(module)
    except Exception as exc:  # noqa: BLE001 - whatever is wrong with the file, say it in one sentence
        sys.modules.pop(private_name, None)
        raise TeamModuleError(f'cannot load {path}: {type(exc).__name__}: {exc}') from exc
    missing = [name for name in REQUIRED_NAMES if not hasattr(module, name)]
    if missing:
        sys.modules.pop(private_name, None)
        raise TeamModuleError(f'{path} no longer offers {", ".join(missing)}: the team code changed; '
                              f'update lattice_bridge/target_locator.py to match')
    _loaded_modules[path] = module
    return module


def default_photo_dir(team_module_path: str) -> str:
    """The photo folder, found EXACTLY the way main_controller finds it.

    main_controller walks up from its own file looking for a folder that holds `install` and `src` (the
    workspace) and uses <that>/src/video_cam/mapping_photos. We start from the main_controller.py that sits
    next to the team module we loaded, so both always end at the same folder, whichever workspace it is.
    """
    current_file = os.path.abspath(os.path.join(os.path.dirname(team_module_path), 'main_controller.py'))
    search_dir = os.path.dirname(current_file)
    workspace = None
    for _ in range(10):
        if os.path.exists(os.path.join(search_dir, 'install')) and os.path.exists(os.path.join(search_dir, 'src')):
            workspace = search_dir
            break
        parent = os.path.dirname(search_dir)
        if os.path.exists(os.path.join(parent, 'install')) and os.path.exists(os.path.join(parent, 'src')):
            workspace = parent
            break
        search_dir = os.path.dirname(search_dir)
        if search_dir == '/':
            break
    if workspace and os.path.exists(os.path.join(workspace, 'src')):
        workspace = os.path.join(workspace, 'src')
    if workspace is None:
        workspace = '/astra/ros2_ws/src'
    return os.path.join(workspace, 'video_cam', 'mapping_photos')


# ---------------------------------------------------------------------------------------------------
# Plain data out of a ROS message
# ---------------------------------------------------------------------------------------------------

class RawDetection(NamedTuple):
    """One detector result: class id ("0" person, "1" tent), score, and the box centre in photo pixels."""
    class_id: str
    score: float
    u: float
    v: float


class PhotoJob(NamedTuple):
    """The answer of the detector for ONE photo, as plain Python (the photo name, the float32 position that
    came in the message, and the detections)."""
    image_name: str
    msg_lat: float
    msg_lon: float
    detections: Tuple[RawDetection, ...]


def _as_float(value) -> float:
    """float(value), or NaN when it is not a number (NaN is later refused by every check)."""
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan


def extract_job(msg, max_detections: int = MAX_DETECTIONS_PER_PHOTO) -> Tuple[PhotoJob, int]:
    """Copy what is needed out of an interfaces/ImageResult (or anything shaped like it).

    Returns (job, malformed): `malformed` counts boxes or results that could not be read. Never raises.
    A box centre or score that is NaN / infinite is KEPT (as is): the node reports it by name instead of
    silently losing it. At most `max_detections` results are taken.
    """
    name = getattr(msg, 'image_name', '')
    name = name if isinstance(name, str) else ''
    name = name[:MAX_NAME_LENGTH + 1]                      # one over the limit, so too-long names stay too long
    lat, lon = _as_float(getattr(msg, 'latitude', math.nan)), _as_float(getattr(msg, 'longitude', math.nan))
    detections = []
    malformed = 0
    try:
        boxes = list(msg.detections.detections)
    except (AttributeError, TypeError):
        boxes = []
        malformed += 1
    for box in boxes:
        try:
            u = float(box.bbox.center.position.x)
            v = float(box.bbox.center.position.y)
            for result in box.results:
                if len(detections) >= max_detections:
                    break
                hypothesis = result.hypothesis
                detections.append(RawDetection(str(hypothesis.class_id), float(hypothesis.score), u, v))
        except (AttributeError, TypeError, ValueError, OverflowError):
            malformed += 1
        if len(detections) >= max_detections:
            break
    return PhotoJob(name, lat, lon, tuple(detections)), malformed


def latlon_from_photo_name(image_name) -> Tuple[Optional[float], Optional[float]]:
    """(lat, lon) from a '<lat> , <lon>.jpg' photo name, or (None, None). Same rule as main_controller."""
    stem = os.path.splitext(os.path.basename(image_name or ''))[0]
    if ' , ' not in stem:
        return None, None
    lat_text, lon_text = stem.split(' , ', 1)
    try:
        return float(lat_text), float(lon_text)
    except ValueError:
        return None, None


def is_plain_file_name(name) -> bool:
    """True for a plain file name: text, 1..255 characters, no folder separators, no NUL, not '.' / '..'."""
    return (isinstance(name, str) and 0 < len(name) <= MAX_NAME_LENGTH and name not in ('.', '..')
            and not any(char in name for char in ('/', '\\', '\x00')))


# ---------------------------------------------------------------------------------------------------
# Locating one detection
# ---------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class LocateResult:
    """Where a detection is on the ground, or why it is not reported.

    ok=True: lat, lon, method ('box' = projected from the box centre, 'photo' = the drone's own position) and
    a note saying how. ok=False: skip_key (one of the SKIP_* words) and skip_text (one sentence).
    """
    ok: bool
    lat: float = math.nan
    lon: float = math.nan
    method: str = ''
    note: str = ''
    skip_key: str = ''
    skip_text: str = ''


def _skipped(key: str, text: str) -> LocateResult:
    return LocateResult(False, skip_key=key, skip_text=text)


class PhotoContext:
    """What is known about one photo: its sidecar, the drone position, the photo size. Read once, lazily.

    `problem_key` / `problem_text` are set when the photo cannot be used at all (bad name, no GPS position).
    """

    def __init__(self, image_name: str):
        self.image_name = image_name
        self.image_path = ''
        self.meta: Optional[dict] = None
        self.sidecar_note = ''
        self.photo_lat: Optional[float] = None
        self.photo_lon: Optional[float] = None
        self.problem_key = ''
        self.problem_text = ''
        self._size: Any = None
        self._size_read = False

    def size(self, geo) -> Optional[Tuple[int, int]]:
        """The photo size in pixels from the JPEG header (read the first time only), or None."""
        if not self._size_read:
            self._size_read = True
            self._size = geo.jpeg_size(self.image_path) if self.image_path else None
        return self._size


class TargetLocator:
    """Turns (photo, box centre) into a position with the team's own code. See the module text."""

    def __init__(self, geo_module, photo_dir: str, aim: Optional[AimSettings] = None,
                 allow_photo_fallback: bool = False):
        self._geo = geo_module
        self.photo_dir = photo_dir
        self._aim = aim or AimSettings()
        self.allow_photo_fallback = bool(allow_photo_fallback)

    @property
    def aim(self) -> AimSettings:
        """The aiming settings in use."""
        return self._aim

    def set_aim(self, aim: AimSettings) -> None:
        """Use other aiming settings from now on (the node does this when main_controller's settings arrive)."""
        self._aim = aim

    def open_photo(self, job: PhotoJob) -> PhotoContext:
        """Read the sidecar and work out the drone position for one photo (steps 1-3 of locate_detection)."""
        geo = self._geo
        context = PhotoContext(job.image_name)
        name = job.image_name
        if name and not is_plain_file_name(name):
            context.problem_key = SKIP_BAD_NAME
            context.problem_text = f'the photo name {name[:40]!r} is not a plain file name'
            return context
        if name:
            context.image_path = os.path.join(self.photo_dir, name)
            context.meta = geo.read_photo_sidecar(context.image_path)
            if context.meta is None:
                exists = os.path.exists(geo.sidecar_path_for(context.image_path))
                context.sidecar_note = ('the sidecar file is unreadable or in another format' if exists
                                        else 'there is no sidecar file for this photo')
        name_lat, name_lon = latlon_from_photo_name(name)
        if name_lat is None:
            name_lat, name_lon = job.msg_lat, job.msg_lon      # main_controller: the float32 copy in the message
        position = geo.photo_position(context.meta, name_lat, name_lon)
        if position is None:
            context.problem_key = SKIP_NO_POSITION
            context.problem_text = 'the photo has no usable GPS position'
            return context
        context.photo_lat, context.photo_lon = position
        return context

    def locate(self, photo: PhotoContext, u, v) -> LocateResult:
        """Where is the box centre (u, v) of this photo on the ground? (steps 4-5 of locate_detection)."""
        if photo.problem_key:
            return _skipped(photo.problem_key, photo.problem_text)
        geo = self._geo
        if not geo.is_finite_number(u, v):
            return _skipped(SKIP_BAD_BOX, f'the box centre ({u}, {v}) is not a finite number')
        aim = self._aim
        if not aim.aim_at_detection_box:
            result = geo.AimResult(photo.photo_lat, photo.photo_lon, 'photo',
                                   note='photo position (aim_at_detection_box is false)')
        else:
            result = geo.aim_at_box(photo.meta, photo.photo_lat, photo.photo_lon, float(u), float(v),
                                    photo.size(geo), default_pitch_deg=aim.default_camera_pitch_deg,
                                    default_hfov_deg=aim.default_camera_hfov_deg,
                                    ground_height_m=aim.target_ground_height_m,
                                    max_offset_m=aim.max_aim_offset_m)
        if result is None:
            return _skipped(SKIP_NO_AIM, 'no usable aim point')
        if not geo.valid_latlon(result.lat, result.lon):
            return _skipped(SKIP_NO_AIM, 'the aim point is not a valid position')
        if result.method != 'box' and not self.allow_photo_fallback:
            why = result.note or 'the box could not be projected'
            if photo.sidecar_note:
                why = f'{why}; {photo.sidecar_note}'
            return _skipped(SKIP_PHOTO_FALLBACK, f'would only be the drone position: {why}')
        return LocateResult(True, float(result.lat), float(result.lon), str(result.method), str(result.note))
