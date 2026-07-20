#!/usr/bin/env python3
"""ortho_mapping configuration constants and the MappingConfig dataclass.

Everything here is ROS-free so the mapper can run offline (CLI) or on a
dev machine with no ROS installed.
"""

from dataclasses import dataclass, field
from typing import Optional

# --- Camera (SIYI A8 mini) ---
# 8 MP stills, 3840x2160 at the '4K' setting, 81 deg horizontal FOV.
DEFAULT_HFOV_DEG = 81.0

# --- Canvas / rendering ---
DEFAULT_GSD_M = 0.03            # 3 cm/px output resolution
DEFAULT_MAX_CANVAS_MP = 120.0   # auto-coarsen GSD if the area needs more px
DEFAULT_FEATHER_FRAC = 0.08     # feather width as a fraction of min(img w, h)
JPEG_QUALITY = 92

# --- Pose fallbacks (used when an image has no .json sidecar) ---
DEFAULT_ALTITUDE_AGL_M = 30.0   # assumed AGL when sidecar lacks rel_alt
MIN_TRACK_STEP_M = 2.0          # min GPS spacing to derive heading from track

# --- Geometry guards ---
# Reject rays that are less than ~10 deg below the horizon (they would
# project kilometres away or never hit the ground plane).
MIN_DOWN_COMPONENT = 0.17
# Reject footprints whose ground span exceeds alt * this factor (a nadir
# 81-deg-HFOV footprint spans ~1.7 * alt, so 8x means "wildly oblique").
MAX_SPAN_FACTOR = 8.0

# --- ECC seam refinement ---
DEFAULT_MAX_REFINE_SHIFT_M = 3.0   # never let ECC move an image further
REFINE_MIN_OVERLAP_FRAC = 0.20     # need >=20% painted overlap to refine
REFINE_MAX_DIM_PX = 512            # ECC runs on a downscaled ROI
REFINE_MAX_ITER = 60
REFINE_EPS = 1e-4

SIDECAR_VERSION = 1
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp')


@dataclass
class MappingConfig:
    """All tunables for one batch mapping run."""
    images_dir: str
    output_dir: str
    output_basename: Optional[str] = None       # default: mapped_<timestamp>
    gsd_m: float = DEFAULT_GSD_M
    hfov_deg: float = DEFAULT_HFOV_DEG
    default_altitude_agl_m: float = DEFAULT_ALTITUDE_AGL_M
    # 'auto'    : sidecar compass heading, else GPS-track bearing
    # 'sidecar' : sidecar compass heading only (skip images without it)
    # 'track'   : always derive heading from the GPS track
    # 'fixed'   : always use fixed_heading_deg
    heading_source: str = 'auto'
    fixed_heading_deg: float = 0.0
    # Extra rotation to correct camera mounting (deg, CW positive). The
    # capture pipeline already applies rotate_180, so this is normally 0.
    yaw_offset_deg: float = 0.0
    use_gimbal_attitude: bool = True   # apply sidecar gimbal pitch/roll/yaw
    refine: bool = True                # ECC seam refinement (GPS-bounded)
    max_refine_shift_m: float = DEFAULT_MAX_REFINE_SHIFT_M
    feather_frac: float = DEFAULT_FEATHER_FRAC
    max_canvas_mp: float = DEFAULT_MAX_CANVAS_MP
    jpeg_quality: int = JPEG_QUALITY
    min_down_component: float = MIN_DOWN_COMPONENT
    max_span_factor: float = MAX_SPAN_FACTOR
    extra: dict = field(default_factory=dict)
