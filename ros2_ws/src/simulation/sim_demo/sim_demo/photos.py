"""
sim_demo/photos.py -- PURPOSE: everything about the team's photo folder: where it is, how
photo files are named, and noticing new photos as they arrive.

Facts this mirrors (from the team's own code, branch aro):
  * The camera node `video_cam siyi` saves every photo into <ros2_ws>/src/video_cam/
    mapping_photos/ (video_cam/storage_manager.py get_ros2_ws_directory() + config.py
    MAPPING_SUBDIR). The detector `detection new_od` and `main main_controller.py` compute
    the same folder independently. (After a flight the team's killnode moves the photos
    to camera_feed/; during a flight they live in mapping_photos/.)
  * The file name carries the drone's GPS position at the moment the shutter fired:
    f"{lat:.6f} , {lon:.6f}.jpg" (space-comma-space). detection/geotag.py reads it back;
    parse_latlon_from_name() below follows exactly the same rules.
  * Files are written as "<name>.tmp.jpg" and then renamed, so half-written files have
    ".tmp." in their name and must be ignored.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass

DELIMITER = ' , '
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp')   # same list as new_od / main_controller
PHOTO_SUBDIR = os.path.join('video_cam', 'mapping_photos')
ANNOTATED_SUBDIR = os.path.join('video_cam', 'detection_results_sahi')


def parse_latlon_from_name(filename: str) -> tuple[float | None, float | None]:
    """Recover (lat, lon) from a '<lat> , <lon>.jpg' photo file name.

    Identical rules to the team's detection/geotag.py, so the panel and the stand-in read
    exactly what the real detector reads.

    Args:
        filename: base name or path of the photo.
    Returns:
        (lat, lon) in degrees, or (None, None) for names without coordinates
        (e.g. 'IMG_0042.jpg' or 'sim_20261010-120000.jpg').
    """
    stem = os.path.splitext(os.path.basename(filename))[0]
    if DELIMITER not in stem:
        return None, None
    lat_str, lon_str = stem.split(DELIMITER, 1)
    try:
        return float(lat_str), float(lon_str)
    except ValueError:
        return None, None


def gps_photo_name(lat: float, lon: float) -> str:
    """The file name video_cam gives a photo taken at (lat, lon): '<lat> , <lon>.jpg'."""
    return f'{lat:.6f} , {lon:.6f}.jpg'


def is_photo_name(name: str) -> bool:
    """True for a finished photo file name (right extension, not a '.tmp.' partial file)."""
    low = name.lower()
    return low.endswith(IMAGE_EXTENSIONS) and '.tmp.' not in low and not low.startswith('.')


def find_ros2_ws_src(start_path: str, max_depth: int = 10) -> str | None:
    """Walk up from start_path to the first folder that has BOTH install/ and src/.

    This is the same search video_cam's get_ros2_ws_directory() does, so it finds the same
    workspace as the camera node when started from inside that workspace.

    Args:
        start_path: a file or folder inside the workspace (e.g. an installed package).
    Returns:
        '<workspace>/src', or None if no workspace was found.
    """
    search = os.path.abspath(start_path)
    if os.path.isfile(search):
        search = os.path.dirname(search)
    for _ in range(max_depth):
        if os.path.isdir(os.path.join(search, 'install')) and os.path.isdir(os.path.join(search, 'src')):
            return os.path.join(search, 'src')
        parent = os.path.dirname(search)
        if parent == search:
            break
        search = parent
    return None


def resolve_photo_dir(override: str = '') -> tuple[str, str]:
    """Find the team's photo folder (where video_cam saves photos and new_od looks for them).

    Order: 1) the `override` argument (a ROS parameter), 2) the ASTRA_PHOTO_DIR environment
    variable, 3) the workspace that holds the installed `video_cam` package (the camera's own
    rule), 4) the Linux-side team repo copy ($ASTRA_REPO, else ~/ODLC_Machine_Inferencing_
    System_2025-2026, if it exists - the same fallback as sim_bringup's sim_paths), 5) the
    workspace that holds this package, 6) video_cam's fallback ~/video_cam/mapping_photos.

    Returns:
        (folder_path, how_it_was_found) -- the folder may not exist yet (it is created by the
        camera node on its first photo).
    """
    if override:
        return os.path.abspath(os.path.expanduser(override)), 'parameter photo_dir'
    env = os.environ.get('ASTRA_PHOTO_DIR', '')
    if env:
        return os.path.abspath(os.path.expanduser(env)), 'environment ASTRA_PHOTO_DIR'
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('video_cam')
        ws_src = find_ros2_ws_src(prefix)
        if ws_src:
            return os.path.join(ws_src, PHOTO_SUBDIR), 'workspace of the video_cam package'
    except Exception:  # noqa: BLE001 - video_cam not built here: try the next rule
        pass
    repo = os.path.expanduser(os.environ.get('ASTRA_REPO', '')
                              or '~/ODLC_Machine_Inferencing_System_2025-2026')
    if os.path.isdir(os.path.join(repo, 'ros2_ws', 'src')):
        return (os.path.join(repo, 'ros2_ws', 'src', PHOTO_SUBDIR),
                'team repo copy ($ASTRA_REPO or ~/ODLC_Machine_Inferencing_System_2025-2026)')
    ws_src = find_ros2_ws_src(os.path.dirname(os.path.abspath(__file__)))
    if ws_src:
        return os.path.join(ws_src, PHOTO_SUBDIR), 'workspace of the sim_demo package'
    return os.path.join(os.path.expanduser('~'), PHOTO_SUBDIR), "video_cam's fallback (home folder)"


def list_photos(folder: str) -> list[tuple[float, str]]:
    """All finished photos in a folder, oldest first.

    Returns:
        list of (modification_time_s, full_path); empty if the folder does not exist.
    """
    out = []
    try:
        with os.scandir(folder) as it:
            for entry in it:
                if entry.is_file() and is_photo_name(entry.name):
                    try:
                        out.append((entry.stat().st_mtime, entry.path))
                    except OSError:
                        continue
    except (FileNotFoundError, NotADirectoryError, PermissionError):
        return []
    out.sort()
    return out


@dataclass
class PhotoEvent:
    """A new, completely written photo.

    Attributes:
        path: full path. name: base name. mtime: modification time (s, wall clock).
        lat, lon: GPS from the file name (None if the name has none).
        size_bytes: file size.
        preexisting: True if the photo was already in the folder when watching started.
    """

    path: str
    name: str
    mtime: float
    lat: float | None
    lon: float | None
    size_bytes: int
    preexisting: bool = False


class PhotoWatcher:
    """Notices new photos in a folder by polling it (no inotify needed, works everywhere).

    Call poll() regularly (e.g. every 0.5 s). A file is reported once, after it is at least
    `min_age_s` old and its size did not change between two polls (= fully written), the
    same idea as new_od's _is_file_ready().
    """

    def __init__(self, folder: str, min_age_s: float = 0.3, report_existing: bool = True):
        """
        Args:
            folder: the folder to watch (may not exist yet).
            min_age_s: minimum age before a file counts as finished.
            report_existing: report photos already present at start (flagged preexisting).
        """
        self.folder = folder
        self.min_age_s = min_age_s
        self._sizes: dict[str, int] = {}
        self._reported: set[str] = set()
        self._started = time.time()
        self._first_poll = True
        self._report_existing = report_existing

    def poll(self, now: float | None = None) -> list[PhotoEvent]:
        """Check the folder once and return the newly finished photos, oldest first."""
        now = time.time() if now is None else now
        events: list[PhotoEvent] = []
        try:
            entries = list(os.scandir(self.folder))
        except (FileNotFoundError, NotADirectoryError, PermissionError):
            return events
        seen_now = set()
        for entry in entries:
            name = entry.name
            if not is_photo_name(name):
                continue
            try:
                st = entry.stat()
            except OSError:
                continue
            if not entry.is_file():
                continue
            seen_now.add(name)
            if name in self._reported:
                continue
            first_poll = self._first_poll
            old_size = self._sizes.get(name)
            self._sizes[name] = st.st_size
            if first_poll:
                # Photos that existed before we started watching.
                self._reported.add(name)
                if self._report_existing and st.st_size > 0:
                    lat, lon = parse_latlon_from_name(name)
                    events.append(PhotoEvent(entry.path, name, st.st_mtime, lat, lon,
                                             st.st_size, preexisting=True))
                continue
            ready = (st.st_size > 0 and old_size == st.st_size
                     and (now - st.st_mtime) >= self.min_age_s)
            if ready:
                self._reported.add(name)
                lat, lon = parse_latlon_from_name(name)
                events.append(PhotoEvent(entry.path, name, st.st_mtime, lat, lon, st.st_size))
        # Forget files that disappeared (so a re-created file with the same name is reported).
        for gone in set(self._sizes) - seen_now:
            self._sizes.pop(gone, None)
            self._reported.discard(gone)
        self._first_poll = False
        events.sort(key=lambda e: e.mtime)
        return events

    @property
    def count_reported(self) -> int:
        """How many photos have been reported so far (including preexisting ones)."""
        return len(self._reported)


def jpeg_size(path: str) -> tuple[int, int] | None:
    """Read an image's (width, height) without decoding all pixels.

    Uses Pillow (python3-pil) when available, else a small JPEG header reader.

    Returns:
        (width, height) or None if the file cannot be read.
    """
    try:
        from PIL import Image  # noqa: PLC0415 - optional dependency
        with Image.open(path) as im:
            return int(im.size[0]), int(im.size[1])
    except Exception:  # noqa: BLE001 - fall back to the manual reader
        pass
    try:
        with open(path, 'rb') as f:
            data = f.read(256 * 1024)
    except OSError:
        return None
    if data[:2] != b'\xff\xd8':
        return None
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            h = (data[i + 5] << 8) | data[i + 6]
            w = (data[i + 7] << 8) | data[i + 8]
            return w, h
        seg_len = (data[i + 2] << 8) | data[i + 3]
        i += 2 + seg_len
    return None
