"""
siyi_emulator/photo.py -- PURPOSE: turn a rendered Gazebo frame into the JPEG the camera stores on its
SD card. It converts the raw gz.msgs.Image bytes to an OpenCV picture, applies the digital zoom
(centre crop + resize back to the photo size, which is exactly what a digital zoom is), encodes the
JPEG (quality ~92), draws a clearly-labelled test pattern when no Gazebo frame is available, and
holds the zoom/field-of-view maths (including the command for ardupilot_gazebo's CameraZoomPlugin
when real optical zoom is enabled).

numpy + OpenCV (cv2), no ROS. Digital-zoom geometry salvaged from the prototype's siyi_sim/zoom.py.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

try:  # OpenCV is required for photos; the rest of the emulator still imports without it.
    import cv2
except ImportError:  # pragma: no cover - only on machines without OpenCV
    cv2 = None

# gz.msgs.PixelFormatType values (gz-msgs10 image.proto) we can convert.
PF_L_INT8 = 1
PF_RGB_INT8 = 3
PF_RGBA_INT8 = 4
PF_BGRA_INT8 = 5
PF_BGR_INT8 = 8

DEFAULT_HFOV_RAD = 1.4137            # 81 deg, the A8 mini lens and the contract's Gazebo cameras
ZOOM_PLUGIN_REF_HFOV_RAD = 2.0       # CameraZoomPlugin divides THIS (not the SDF HFOV) by the command


def _require_cv2() -> None:
    if cv2 is None:  # pragma: no cover
        raise RuntimeError("OpenCV (python3-opencv) is required to make photos")


# ---------------------------------------------------------------------------------------------
# Gazebo image -> OpenCV BGR
# ---------------------------------------------------------------------------------------------
def gz_image_to_bgr(width: int, height: int, step: int, data: bytes, pixel_format: int) -> np.ndarray:
    """Convert the fields of a gz.msgs.Image into an OpenCV BGR uint8 array (H x W x 3).

    Args:
        width, height: image size in pixels.
        step:          bytes per row (0 = width * channels).
        data:          raw pixel bytes.
        pixel_format:  gz.msgs.PixelFormatType (RGB_INT8 = 3 is what Gazebo cameras send for R8G8B8).

    Raises:
        ValueError: unsupported pixel format or too few bytes.
    """
    channels = {PF_L_INT8: 1, PF_RGB_INT8: 3, PF_BGR_INT8: 3, PF_RGBA_INT8: 4, PF_BGRA_INT8: 4}.get(int(pixel_format))
    if channels is None:
        raise ValueError(f"unsupported gz pixel format {pixel_format} (expected RGB_INT8 = 3)")
    width, height = int(width), int(height)
    row = int(step) if step else width * channels
    if width <= 0 or height <= 0 or row < width * channels:
        raise ValueError(f"bad image geometry {width}x{height} step {step}")
    buf = np.frombuffer(data, dtype=np.uint8)
    if buf.size < row * height:
        raise ValueError(f"image data too short ({buf.size} bytes, need {row * height})")
    img = buf[: row * height].reshape(height, row)[:, : width * channels].reshape(height, width, channels)
    if channels == 1:
        return np.repeat(img, 3, axis=2)
    if int(pixel_format) == PF_RGB_INT8:
        return img[:, :, ::-1].copy()                  # RGB -> BGR
    if int(pixel_format) == PF_BGR_INT8:
        return img.copy()
    if int(pixel_format) == PF_RGBA_INT8:
        return img[:, :, 2::-1].copy()                 # RGBA -> BGR
    return img[:, :, :3].copy()                        # BGRA -> BGR


# ---------------------------------------------------------------------------------------------
# Zoom geometry
# ---------------------------------------------------------------------------------------------
def crop_rect(width: int, height: int, zoom: float) -> Tuple[int, int, int, int]:
    """Centre crop (x0, y0, w, h) for a digital zoom factor; even sizes, at least 2x2 pixels."""
    zoom = max(1.0, float(zoom))
    w = max(2, int(round(width / zoom)) // 2 * 2)
    h = max(2, int(round(height / zoom)) // 2 * 2)
    w, h = min(w, int(width)), min(h, int(height))
    return (int(width) - w) // 2, (int(height) - h) // 2, w, h


def apply_digital_zoom(img: np.ndarray, zoom: float, out_size: Optional[Tuple[int, int]] = None) -> np.ndarray:
    """Digital zoom: keep the centre 1/zoom of the picture and resize it to ``out_size`` (w, h).

    With zoom 1.0 and the same size the input is returned unchanged.
    """
    _require_cv2()
    h, w = img.shape[:2]
    out_w, out_h = (int(out_size[0]), int(out_size[1])) if out_size else (w, h)
    x0, y0, cw, ch = crop_rect(w, h, zoom)
    if (x0, y0, cw, ch) == (0, 0, w, h) and (w, h) == (out_w, out_h):
        return img
    crop = img[y0: y0 + ch, x0: x0 + cw]
    interp = cv2.INTER_AREA if (out_w < cw or out_h < ch) else cv2.INTER_LINEAR
    return cv2.resize(crop, (out_w, out_h), interpolation=interp)


def split_zoom(total_zoom: float, optical_max: float) -> Tuple[float, float]:
    """Split a zoom factor into (optical, digital) parts: optical up to ``optical_max``, the rest digital."""
    total = max(1.0, float(total_zoom))
    optical = min(total, max(1.0, float(optical_max)))
    return optical, total / optical


def zoomed_hfov(hfov_rad: float, zoom: float) -> float:
    """Horizontal FOV after a zoom of ``zoom`` (true for both crop and real optics: tan scales by 1/zoom)."""
    return 2.0 * math.atan(math.tan(float(hfov_rad) / 2.0) / max(1.0, float(zoom)))


def zoom_plugin_command(optical_zoom: float, hfov_rad: float = DEFAULT_HFOV_RAD,
                        ref_hfov_rad: float = ZOOM_PLUGIN_REF_HFOV_RAD) -> float:
    """Value to publish to ardupilot_gazebo's CameraZoomPlugin for a real optical zoom factor.

    The plugin sets HFOV = ref_hfov / command with a hard-coded ref_hfov of 2.0 rad (not the camera's
    own HFOV) and it also drives the camera to 2.0 rad at start if it never gets a command. To get the
    physically correct HFOV = 2*atan(tan(hfov/2)/zoom) we therefore send ref_hfov / that HFOV.
    Example: hfov 1.4137 rad, zoom 1 -> 1.4147; zoom 5 -> 5.910.
    """
    return float(ref_hfov_rad) / zoomed_hfov(hfov_rad, optical_zoom)


def hfov_from_camera_info(width: int, fx: float) -> Optional[float]:
    """Horizontal FOV (rad) from image width and focal length fx in pixels, or None if invalid."""
    if width <= 0 or fx <= 0:
        return None
    return 2.0 * math.atan(width / (2.0 * fx))


# ---------------------------------------------------------------------------------------------
# JPEG and test pattern
# ---------------------------------------------------------------------------------------------
def encode_jpeg(img: np.ndarray, quality: int = 92) -> bytes:
    """Encode a BGR image as JPEG bytes (quality 1..100)."""
    _require_cv2()
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(max(1, min(100, quality)))])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return buf.tobytes()


def make_test_pattern(width: int, height: int, lines: Sequence[str] = (), seed: int = 0) -> np.ndarray:
    """A textured, clearly-labelled stand-in picture used when Gazebo sends no photo frame.

    Random noise + a grid + big text, so it is obviously NOT a real view, decodes like any photo and
    stays well above video_cam's minimum JPEG size (50 kB for the 4K profile) after re-encoding.
    """
    _require_cv2()
    rng = np.random.default_rng(seed)
    img = rng.integers(40, 200, size=(int(height), int(width), 3), dtype=np.uint8)
    img[:, :, 1] = np.clip(img[:, :, 1].astype(np.int16) + 30, 0, 255).astype(np.uint8)
    step = max(32, int(width) // 16)
    for x in range(0, int(width), step):
        cv2.line(img, (x, 0), (x, int(height) - 1), (255, 255, 255), 2)
    for y in range(0, int(height), step):
        cv2.line(img, (0, y), (int(width) - 1, y), (255, 255, 255), 2)
    cv2.circle(img, (int(width) // 2, int(height) // 2), max(10, int(height) // 6), (0, 0, 255), 6)
    scale = max(0.8, int(width) / 900.0)
    texts = ["SIYI EMULATOR TEST PATTERN", "NO GAZEBO PHOTO FRAME"] + list(lines)
    y = int(height * 0.12)
    for text in texts:
        cv2.putText(img, text, (int(width * 0.04), y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), int(scale * 6))
        cv2.putText(img, text, (int(width * 0.04), y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 255), int(scale * 2))
        y += int(scale * 45)
    return img


def resize_to(img: np.ndarray, size: Tuple[int, int]) -> np.ndarray:
    """Resize to (w, h) if needed (keeps the input when it already has that size)."""
    _require_cv2()
    w, h = int(size[0]), int(size[1])
    if img.shape[1] == w and img.shape[0] == h:
        return img
    interp = cv2.INTER_AREA if img.shape[1] > w else cv2.INTER_LINEAR
    return cv2.resize(img, (w, h), interpolation=interp)
