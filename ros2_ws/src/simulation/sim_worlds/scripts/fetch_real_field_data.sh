#!/usr/bin/env bash
# sim_worlds/scripts/fetch_real_field_data.sh -- PURPOSE: download (once) the real, public data the
# "suas_field_real" world is made from - USGS elevation + lidar, the USDA NAIP aerial photo, OpenStreetMap,
# Microsoft building footprints, CC0 ground textures - into ~/.astra_sim/world_data/suas_field_real/raw/ and
# check every file against the checksums in sim_worlds/data/real_field_manifest.json.
#
# You only need this to RE-GENERATE the world (scripts/build_real_field.py prepare/build). Just FLYING in the
# world needs nothing: its finished assets are committed in models/suas_field_real_*.
#
# The real work is done by fetch_real_field_data.py next to this file; this wrapper picks a Python that has the
# GIS libraries (rasterio is needed for the photo windows) and can create one for you.
#
# usage:
#   fetch_real_field_data.sh                    required layers (about 400 MB: lidar 271 MB, photo, DTM ...)
#   fetch_real_field_data.sh --optional         + optional layers (leaf-off photo 165 MB, Meta canopy, Sentinel-2)
#   fetch_real_field_data.sh --check            only verify what is already downloaded (no network)
#   fetch_real_field_data.sh --list             print every source with its licence
#   fetch_real_field_data.sh --only naip,osm    some layers only
#   fetch_real_field_data.sh --from DIR         copy from a folder that already has the files (still verified)
#   fetch_real_field_data.sh --setup-venv       create ~/.astra_sim/gis_venv (rasterio, pyproj, laspy, scipy ...)
# environment:  ASTRA_WORLD_DATA=/other/folder   (default ~/.astra_sim/world_data)
#               GIS_PY=/path/to/python           (default: ~/.astra_sim/gis_venv/bin/python if it exists)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${HOME}/.astra_sim/gis_venv"

if [ "${1:-}" = "--setup-venv" ]; then
  # A separate Python environment for the GIS libraries, so ROS's own Python packages stay untouched.
  echo "creating ${VENV} (needs python3-venv: sudo apt install python3-venv) ..."
  python3 -m venv "${VENV}"
  "${VENV}/bin/pip" install --upgrade pip
  "${VENV}/bin/pip" install "rasterio>=1.3" "pyproj>=3.4" "laspy[lazrs]>=2.4" "scipy>=1.8" "numpy>=1.22" \
      "pillow>=9" "pyyaml>=5.4"
  echo "done. Use it with:  GIS_PY=${VENV}/bin/python  (the scripts pick it up automatically)"
  exit 0
fi

PY="${GIS_PY:-}"
if [ -z "${PY}" ]; then
  if [ -x "${VENV}/bin/python" ]; then PY="${VENV}/bin/python"; else PY="$(command -v python3)"; fi
fi
if ! "${PY}" -c "import rasterio" >/dev/null 2>&1; then
  echo "note: ${PY} has no rasterio: plain downloads work, the photo/canopy WINDOWS (naip, metachm, s2) will fail."
  echo "      Fix: bash $0 --setup-venv   (one time, about 2 minutes)"
fi
# -I = isolated Python: files inside the download folders can never be imported by accident
exec "${PY}" -I "${HERE}/fetch_real_field_data.py" "$@"
