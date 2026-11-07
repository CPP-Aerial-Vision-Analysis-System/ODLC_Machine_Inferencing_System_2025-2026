#!/usr/bin/env bash
# =============================================================================
#  upscale_env.sh  --  OPTIONAL: set up the picture sharpener for the real-terrain world
#
#  PURPOSE: the aerial photo the real-terrain world (suas_field_real) is painted from is blurry when the drone camera looks
#  at it from 50 m. This script makes a SEPARATE Python environment (~/.astra_sim/upscale_venv) with the Real-ESRGAN picture
#  upscaler and downloads its model file (67 MB, BSD-3 licence) to ~/.astra_sim/models/RealESRGAN_x4plus.pth. It reuses the
#  PC's PyTorch (the one the detector uses), so it needs a PyTorch with CUDA already installed, and it never changes ROS's
#  Python. Afterwards:
#      ~/.astra_sim/upscale_venv/bin/python scripts/build_real_field.py upscale   (from ros2_ws/src/simulation/sim_worlds; about 25 s on a GPU)
#      python3 scripts/build_real_field.py assets --install (paints the ground tiles from the sharper picture)
#  Without this step everything still works; the ground is just softer. See docs/simulation/WORLDS.md "The optional sharper
#  ground".
#
#  HOW TO RUN:  bash simulation/setup/upscale_env.sh        (no sudo; needs internet once)
#  Exit codes:  0 = ready, 1 = failed (the reason and the fix are printed)
# =============================================================================
set -u
VENV="$HOME/.astra_sim/upscale_venv"
WEIGHTS="$HOME/.astra_sim/models/RealESRGAN_x4plus.pth"
say() { echo "[upscale_env] $*"; }
die() { echo "[upscale_env] ERROR: $*" >&2; exit 1; }

python3 -c "import torch, torchvision" 2>/dev/null \
  || die "PyTorch is not installed for python3. Likely cause: the simulator setup (01_install_humble.sh) was not run. Fix: run it, or pip install torch torchvision."
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV" || die "could not make a virtual environment. Fix: sudo apt-get install -y python3-venv"
# shellcheck disable=SC1091
. "$VENV/bin/activate"
say "installing Real-ESRGAN (a few minutes the first time)"
pip install --quiet realesrgan basicsr facexlib gfpgan opencv-python-headless || die "pip install failed. Likely cause: no internet. Fix: connect and run this again."
# basicsr 1.4.2 imports a torchvision module that newer torchvision removed; point it at the new place
DEG="$(ls "$VENV"/lib/python3*/site-packages/basicsr/data/degradations.py 2>/dev/null | head -1)"
[ -n "$DEG" ] && sed -i 's/from torchvision.transforms.functional_tensor import rgb_to_grayscale/from torchvision.transforms.functional import rgb_to_grayscale/' "$DEG"
python -c "import basicsr, realesrgan" 2>/dev/null || die "Real-ESRGAN does not import after the install. Fix: read the pip output above."
mkdir -p "$(dirname "$WEIGHTS")"
if [ ! -s "$WEIGHTS" ]; then
  say "downloading the model file (67 MB)"
  curl -L -s -o "$WEIGHTS" https://github.com/xinntao/Real-ESRGAN/releases/download/v0.1.0/RealESRGAN_x4plus.pth \
    || die "download failed. Fix: connect to the internet and run this again."
fi
[ -s "$WEIGHTS" ] || die "the model file is empty. Fix: delete $WEIGHTS and run this again."
say "ready. Next: python3 scripts/build_real_field.py upscale   (use the venv's python: $VENV/bin/python)"
say "note: the upscale command needs torch, so run it with  $VENV/bin/python scripts/build_real_field.py upscale"
