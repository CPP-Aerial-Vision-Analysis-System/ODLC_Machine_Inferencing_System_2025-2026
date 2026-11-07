#!/bin/bash
# =============================================================================
#  ~/astra_hil/bin/astra_hil.sh  --  run the team container ON the Jetson without the PC (simulation/jetson/lib/astra_hil.sh)
#
#  PURPOSE: on the drone there is no PC and no repo checkout: this small script (copied to ~/astra_hil/bin/ by
#  `jetson.sh setup`) starts, stops and shows the team's container directly on the Jetson. REAL mode is what the
#  drone runs: the team's ./launcher.sh unchanged inside the container, host network, MAVROS on /dev/ttyUSB0, the real
#  SIYI camera on eth0, the team's own photo folders. The native ./launcher.sh (no Docker) keeps working exactly as
#  today; this container is an option. Only containers labelled org.astra.owner=hil are ever touched.
#  Usage (on the Jetson):  ~/astra_hil/bin/astra_hil.sh start real [--archive-photos|--keep-photos] [--domain N]
#                          ~/astra_hil/bin/astra_hil.sh stop | status | logs [-f]
# =============================================================================
D="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEAM_ROOT="${TEAM_ROOT:-$HOME/astra}"
PHOTOS="$TEAM_ROOT/ros2_ws/src/video_cam/mapping_photos"
DETECT="$TEAM_ROOT/ros2_ws/src/video_cam/detection_results_sahi"
W=/home/astra-dev/astra/ros2_ws/src/video_cam
if docker version >/dev/null 2>&1; then DK=docker; else DK="sudo -n docker"; fi
err() { printf '[astra_hil] ERROR: %s\n' "$*" >&2; exit 1; }

case "${1:-}" in
  start)
    [ "${2:-}" = real ] || err "only 'start real' works without the PC (SIM mode needs the PC's link: run it from the PC with jetson.sh start --sim)."
    shift 2; photos=refuse; dom=0
    while [ $# -gt 0 ]; do
      case "$1" in --archive-photos) photos=archive ;; --keep-photos) photos=keep ;; --domain) dom="$2"; shift ;; esac; shift
    done
    pgrep -f 'mavros_node|tracker.launch' >/dev/null && err "the team's native stack (MAVROS / tracker) is running. Likely cause: launcher.sh started at boot. Both would open /dev/ttyUSB0. Fix: stop the native stack first, then run this again."
    if [ -n "$(ls -A "$PHOTOS" 2>/dev/null)" ]; then
      case "$photos" in
        refuse) err "the photo folder $PHOTOS is not empty. Old photos would be processed again and could send the drone to an old target. Fix: add --archive-photos (moves them to $D/photo_archive/) or --keep-photos" ;;
        archive) a="$D/photo_archive/$(date +%Y%m%d-%H%M%S)_real"; mkdir -p "$a" && mv "$PHOTOS"/* "$a"/ && echo "archived old photos to $a" ;;
      esac
    fi
    $DK rm -f astra-hil-real >/dev/null 2>&1
    $DK run -d --init --name astra-hil-real --label org.astra.owner=hil --runtime nvidia --network host --ipc host \
      --device /dev/ttyUSB0 --group-add dialout -e ASTRA_HIL_MODE=real -e ASTRA_UID="$(id -u)" -e ASTRA_GID="$(id -g)" \
      -e ROS_DOMAIN_ID="$dom" -v "$D/models:/models" -v "$D/runs:/data/runs" \
      -v "$PHOTOS:$W/mapping_photos" -v "$DETECT:$W/detection_results_sahi" astra-hil:jetson real \
      || err "docker run failed (see above). Fix: is the image deployed (docker images astra-hil)? Is the Pixhawk on /dev/ttyUSB0?"
    echo "started astra-hil-real. Logs: $0 logs -f"
    ;;
  stop)
    for c in astra-hil-real astra-hil astra-hil-detector; do
      [ "$($DK inspect -f '{{index .Config.Labels "org.astra.owner"}}' "$c" 2>/dev/null)" = hil ] || continue
      $DK stop -t 25 "$c" >/dev/null && $DK rm "$c" >/dev/null && echo "stopped $c"
    done
    ;;
  status)
    $DK ps -a --filter label=org.astra.owner=hil --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    ;;
  logs)
    shift; c=astra-hil-real
    $DK inspect astra-hil >/dev/null 2>&1 && ! $DK inspect astra-hil-real >/dev/null 2>&1 && c=astra-hil
    $DK logs --tail 200 "$@" "$c"
    ;;
  *)
    sed -n '2,14p' "$0"; exit 2 ;;
esac
