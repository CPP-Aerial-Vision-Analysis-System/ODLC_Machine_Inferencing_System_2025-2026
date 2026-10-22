#!/usr/bin/env bash
# =============================================================================
#  take_photo.sh  --  take ONE photo with the simulated drone camera ("press the shutter")
#
#  PURPOSE: the simulator's photo button for the terminal. It runs `ros2 run sim_bringup photo_trigger`:
#    * normally it publishes `true` on /camera/trigger, the team's own topic: the team's video_cam then asks
#      the camera (the SIYI emulator) for a photo, Gazebo renders it, and video_cam saves it in the team's
#      photo folder as "<lat> , <lon>.jpg" (needs ./launcher.sh sim running);
#    * when nobody listens on /camera/trigger (team stack not running), or with --direct, it sends the SIYI
#      "take photo" command straight to the emulator and downloads the photo into ~/.astra_sim/photos.
#  It prints the path of the new photo.
#
#  HOW TO RUN:   bash take_photo.sh               (simulator running, start_sim.sh or demo.sh)
#                bash take_photo.sh --direct      (skip the team code, talk to the camera emulator)
#  Options are passed to photo_trigger: --timeout SECONDS, --camera-ip ADDRESS, --photo-dir DIR.
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
case "${1:-}" in -h|--help) astra_usage "$0"; exit 0 ;; esac
astra_load_env
astra_require_built
say "Taking a photo with the simulated camera"
ros2 run sim_bringup photo_trigger "$@"
