"""
siyi_emulator/sd_card.py -- PURPOSE: the emulated micro SD ("TF") card of the camera. Photos are real
JPEG files in one folder on the computer:

    <sd_card_dir>/<photo_dir_name>/IMG_0001.jpg      the photo (what the media server lists)
    <sd_card_dir>/<photo_dir_name>/IMG_0001.json     metadata sidecar (camera pose, zoom ...; not listed)

Why one folder for the whole run: video_cam only watches the LAST directory returned by
getdirectories and never re-lists directories (video_cam.md section 4.4 / 17), so a camera that
opened a new folder mid-flight would make every capture time out. The default folder name
"101SIYI_IMG" is what a real SIYI camera answered (video_cam/siyiInstructions.md:9-16).

Numbering never reuses a name during a run, even after a format (0x48), because video_cam remembers
every name it has already seen; ``restart_numbering_after_format=True`` reproduces a camera that
starts again at IMG_0001 (see the README, "known hazards").

Pure Python + files. Thread-safe (one lock around numbering, writing and deleting).
"""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

PHOTO_PREFIX = "IMG_"
_PHOTO_RE = re.compile(r"^IMG_(\d{4,})\.jpe?g$", re.IGNORECASE)
_SIDECAR_RE = re.compile(r"^IMG_(\d{4,})\.json$", re.IGNORECASE)


@dataclass(frozen=True)
class PhotoFile:
    """One photo on the card.

    Attributes:
        index: the NNNN number of IMG_NNNN.jpg.
        name:  file name, e.g. 'IMG_0007.jpg'.
        path:  absolute path on this computer.
        size:  file size in bytes.
        rel_path: '<photo_dir_name>/<name>', the path part of its download URL.
    """

    index: int
    name: str
    path: str
    size: int
    rel_path: str


def default_sd_card_dir(value: str = "") -> str:
    """Resolve the sd_card_dir parameter: '~' and $VARS expanded; empty = ~/.ros/siyi_emulator/sd_card."""
    value = (value or "").strip() or "~/.ros/siyi_emulator/sd_card"
    return os.path.abspath(os.path.expanduser(os.path.expandvars(value)))


class SdCard:
    """Folder-backed emulation of the camera's micro SD card.

    Args:
        root_dir:        card root folder (created if needed).
        dir_name:        the single photo directory reported by getdirectories (default '101SIYI_IMG').
        clear_on_start:  True = delete old photos at start (a freshly formatted card for each run).
        restart_numbering_after_format: True = IMG_0001 again after 0x48 (see module docstring).
    """

    def __init__(self, root_dir: str, dir_name: str = "101SIYI_IMG", clear_on_start: bool = True,
                 restart_numbering_after_format: bool = False) -> None:
        self.root = Path(default_sd_card_dir(root_dir))
        dir_name = (dir_name or "101SIYI_IMG").strip().strip("/\\")
        if not dir_name or ".." in dir_name or "/" in dir_name or "\\" in dir_name:
            raise ValueError(f"photo_dir_name must be a plain folder name, got '{dir_name}'")
        self.dir_name = dir_name
        self.restart_numbering_after_format = bool(restart_numbering_after_format)
        self._lock = threading.Lock()
        self.photo_dir.mkdir(parents=True, exist_ok=True)
        if clear_on_start:
            self._delete_all_locked()
        self._next_index = self._highest_index_on_disk() + 1

    # ------------------------------------------------------------------ paths
    @property
    def photo_dir(self) -> Path:
        """Absolute path of the photo directory."""
        return self.root / self.dir_name

    def directories(self) -> List[Dict[str, str]]:
        """What getdirectories returns: always exactly one entry, ``[{"name": d, "path": d}]``."""
        return [{"name": self.dir_name, "path": self.dir_name}]

    def has_directory(self, path: str) -> bool:
        """True when ``path`` (from a getmedialist request) means our photo directory.

        Leading/trailing slashes are ignored, so '101SIYI_IMG', '/101SIYI_IMG/' both match.
        """
        return (path or "").strip().strip("/\\") == self.dir_name

    # ------------------------------------------------------------------ listing
    def _highest_index_on_disk(self) -> int:
        best = 0
        if self.photo_dir.is_dir():
            for entry in self.photo_dir.iterdir():
                m = _PHOTO_RE.match(entry.name) or _SIDECAR_RE.match(entry.name)
                if m:
                    best = max(best, int(m.group(1)))
        return best

    def list_photos(self) -> List[PhotoFile]:
        """Every IMG_NNNN.jpg in the photo directory, OLDEST FIRST (by number).

        video_cam walks the list from the end to find the newest photo, so the order matters.
        Files that disappear while listing are skipped.
        """
        photos: List[PhotoFile] = []
        if not self.photo_dir.is_dir():
            return photos
        for entry in self.photo_dir.iterdir():
            m = _PHOTO_RE.match(entry.name)
            if not m:
                continue
            try:
                size = entry.stat().st_size
            except OSError:
                continue
            photos.append(PhotoFile(int(m.group(1)), entry.name, str(entry), size, f"{self.dir_name}/{entry.name}"))
        photos.sort(key=lambda p: p.index)
        return photos

    def count(self) -> int:
        """Number of photos on the card."""
        return len(self.list_photos())

    def resolve(self, rel_path: str) -> Optional[Path]:
        """Map a URL path like '101SIYI_IMG/IMG_0001.jpg' to a file inside the card, safely.

        Returns None for anything outside the card (e.g. '../../etc/passwd') or missing files.
        """
        rel = (rel_path or "").replace("\\", "/").strip("/")
        if not rel or any(part in ("..", "") for part in rel.split("/")):
            return None
        candidate = (self.root / rel).resolve()
        try:
            candidate.relative_to(self.root.resolve())
        except ValueError:
            return None
        return candidate if candidate.is_file() else None

    # ------------------------------------------------------------------ writing
    def peek_next_name(self) -> str:
        """Name the next photo will get (does not reserve it)."""
        with self._lock:
            return f"{PHOTO_PREFIX}{self._next_index:04d}.jpg"

    def write_photo(self, jpeg_bytes: bytes, sidecar: Optional[Dict[str, Any]] = None) -> PhotoFile:
        """Store one photo (and its JSON sidecar) atomically, under the next free IMG_NNNN name.

        Atomic = written to a hidden temp file first, then renamed, so the media server never
        lists or serves a half-written JPEG.

        Args:
            jpeg_bytes: the encoded JPEG.
            sidecar:    optional dict saved as IMG_NNNN.json ("file" and "index" keys are added).

        Returns:
            The :class:`PhotoFile` of the new photo.
        """
        with self._lock:
            self.photo_dir.mkdir(parents=True, exist_ok=True)
            index = self._next_index
            self._next_index += 1
            name = f"{PHOTO_PREFIX}{index:04d}.jpg"
            path = self.photo_dir / name
            if sidecar is not None:
                meta = dict(sidecar)
                meta["file"] = f"{self.dir_name}/{name}"
                meta["index"] = index
                _atomic_write(self.photo_dir / f"{PHOTO_PREFIX}{index:04d}.json",
                              json.dumps(meta, indent=2, default=_json_default).encode("utf-8"))
            _atomic_write(path, bytes(jpeg_bytes))
            return PhotoFile(index, name, str(path), len(jpeg_bytes), f"{self.dir_name}/{name}")

    # ------------------------------------------------------------------ formatting
    def _delete_all_locked(self) -> int:
        deleted = 0
        if self.photo_dir.is_dir():
            for entry in self.photo_dir.iterdir():
                try:
                    if entry.is_dir():
                        shutil.rmtree(entry)
                    else:
                        entry.unlink()
                    deleted += 1
                except OSError:
                    pass
        return deleted

    def format(self) -> int:
        """0x48: delete every file in the photo directory (the directory itself stays).

        Returns:
            Number of files deleted.
        """
        with self._lock:
            deleted = self._delete_all_locked()
            if self.restart_numbering_after_format:
                self._next_index = 1
            return deleted


def _json_default(obj: Any) -> Any:
    """json.dumps helper for numpy numbers/arrays without importing numpy here."""
    if hasattr(obj, "tolist"):
        return obj.tolist()
    if hasattr(obj, "item"):
        return obj.item()
    return str(obj)


def _atomic_write(path: Path, data: bytes) -> None:
    """Write ``data`` to ``path`` through a temp file + rename (never a half-written file)."""
    fd, tmp = tempfile.mkstemp(prefix=".tmp_", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
        os.chmod(tmp, 0o644)        # mkstemp makes 0600; photos should be readable like normal files
        os.replace(tmp, path)
    except Exception:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise
