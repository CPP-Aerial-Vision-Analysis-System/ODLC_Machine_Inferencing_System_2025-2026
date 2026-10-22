"""
sim_demo/zoom.py -- PURPOSE: know how far the simulated camera was zoomed in when each photo
was taken. Zoom matters a lot: last year's camera code racks the lens to 5x at the start of
the search (video_cam config CAPTURE_ZOOM_X = 5.0, triggered by "capture_setup" on the
DigiCamCtrl mission item), and at 5x a photo covers 5 times less ground in each direction.

Sources, best first:
  1. a fixed value given as a parameter (zoom > 0) - for testing;
  2. the camera emulator's own report (its per-photo JSON sidecar, or a zoom topic);
  3. the team camera node's status text "Zoom set to 5.0x" (sent after it confirmed the zoom);
  4. otherwise 1.0 (no zoom).
Pure Python; keeps a short history so the zoom at any past moment can be looked up.
"""
from __future__ import annotations

import bisect
import re

_RE_ZOOM_SET = re.compile(r'Zoom set to\s+([0-9.]+)\s*x', re.IGNORECASE)
_RE_ZOOM_CMD = re.compile(r'^\s*zoom_(?:absolute|auto)\s+([0-9.]+)\s*$', re.IGNORECASE)


class ZoomTracker:
    """Zoom factor over time from several sources (see module docstring)."""

    SOURCE_FIXED = 'parameter'
    SOURCE_EMULATOR = 'camera emulator'
    SOURCE_STATUS = 'video_cam status text'
    SOURCE_COMMAND = '/camera/command'
    SOURCE_DEFAULT = 'default (no zoom reported)'

    def __init__(self, fixed_zoom: float = 0.0):
        """
        Args:
            fixed_zoom: > 0 forces this zoom for every photo (ignores all sources).
        """
        self.fixed_zoom = float(fixed_zoom or 0.0)
        self._emu_t: list[float] = []
        self._emu_z: list[float] = []
        self._txt_t: list[float] = []
        self._txt_z: list[tuple[float, str]] = []

    @staticmethod
    def _add(ts: list, vs: list, t: float, value) -> None:
        i = bisect.bisect_right(ts, t)
        ts.insert(i, t)
        vs.insert(i, value)
        if len(ts) > 5000:
            del ts[:1000]
            del vs[:1000]

    def on_emulator_zoom(self, t: float, zoom: float) -> None:
        """Record a zoom value reported by the camera emulator at time t."""
        if zoom and zoom > 0:
            self._add(self._emu_t, self._emu_z, t, float(zoom))

    def on_statustext(self, t: float, text: str) -> bool:
        """Parse a status text; True if it carried a zoom ("Zoom set to 5.0x")."""
        m = _RE_ZOOM_SET.search(text or '')
        if not m:
            return False
        self._add(self._txt_t, self._txt_z, t, (float(m.group(1)), self.SOURCE_STATUS))
        return True

    def on_camera_command(self, t: float, text: str) -> bool:
        """Parse a /camera/command string; True if it was an absolute zoom command."""
        m = _RE_ZOOM_CMD.match(text or '')
        if not m:
            return False
        self._add(self._txt_t, self._txt_z, t, (float(m.group(1)), self.SOURCE_COMMAND))
        return True

    def zoom_at(self, t: float) -> tuple[float, str]:
        """Zoom factor in effect at time t.

        Returns:
            (zoom, source_description)
        """
        if self.fixed_zoom > 0:
            return self.fixed_zoom, self.SOURCE_FIXED
        if self._emu_t:
            i = bisect.bisect_right(self._emu_t, t)
            return self._emu_z[max(0, i - 1)], self.SOURCE_EMULATOR
        if self._txt_t:
            i = bisect.bisect_right(self._txt_t, t)
            if i > 0:
                z, src = self._txt_z[i - 1]
                return z, src
        return 1.0, self.SOURCE_DEFAULT
