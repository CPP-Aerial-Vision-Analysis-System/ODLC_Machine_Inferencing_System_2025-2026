#!/usr/bin/env bash
# =============================================================================
#  install_blender.sh  --  install Blender on WINDOWS from the Ubuntu terminal
#
#  PURPOSE: Blender is the tool that OPENS the real-terrain world for inspection (a .blend you can fly around in, and test renders of what the
#  drone camera sees). It is NOT needed to build or fly the world: the build is numpy + Pillow and works on a PC without Blender. This script runs
#  Windows' package manager (winget) through WSL to install Blender 5.x (tested with 5.2.3). Windows may show an administrator prompt: say yes.
#
#  HOW TO RUN:  bash simulation/setup/install_blender.sh        (no sudo)
#  AFTER:       python3 ros2_ws/src/simulation/sim_worlds/scripts/build_real_field.py blender --check
#               expect:  [blender] found: ... version 5.2.x
#  By hand instead (Windows PowerShell):   winget install --id BlenderFoundation.Blender -e
#  Or download it from https://www.blender.org/download/ . If it is installed somewhere unusual:  export BLENDER_EXE="/mnt/d/.../blender.exe"
#  Exit codes:  0 = installed (or already there), 1 = could not install (the reason and the fix are printed)
# =============================================================================
set -u
say() { echo "[install_blender] $*"; }
die() { echo "[install_blender] ERROR: $*" >&2; exit 1; }
command -v powershell.exe >/dev/null 2>&1 || die "this is not WSL on Windows (powershell.exe not found). Fix: install Blender 5.x from https://www.blender.org/download/ on the Windows side."
if ls /mnt/*/Program\ Files/Blender\ Foundation/Blender\ */blender.exe >/dev/null 2>&1; then
  say "Blender is already installed: $(ls -d /mnt/*/Program\ Files/Blender\ Foundation/Blender\ */ | tail -1)"
  exit 0
fi
say "installing Blender with winget (a few minutes; accept the Windows prompt)"
powershell.exe -NoProfile -Command "winget install --id BlenderFoundation.Blender -e --accept-package-agreements --accept-source-agreements" \
  || die "winget failed. Likely cause: winget is missing (old Windows) or you declined the prompt. Fix: install Blender 5.x by hand from https://www.blender.org/download/, then run: python3 build_real_field.py blender --check"
say "done. Check:  python3 ros2_ws/src/simulation/sim_worlds/scripts/build_real_field.py blender --check"
