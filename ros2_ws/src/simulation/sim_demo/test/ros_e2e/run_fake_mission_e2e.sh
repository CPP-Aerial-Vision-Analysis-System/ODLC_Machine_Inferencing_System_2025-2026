#!/usr/bin/env bash
# sim_demo/test/ros_e2e/run_fake_mission_e2e.sh -- PURPOSE: end-to-end test of sim_demo's four
# nodes against a scripted fake mission (no Gazebo, no SITL): starts truth_detector,
# demo_markers, score_run and demo_panel (offscreen, taking screenshots), plays
# fake_mission.py (~90 s), then checks: every photo got an /image_detection result, the TEAM'S
# main_controller callback chose a person and a tent, all RViz marker layers were published,
# the score report was written with both targets found, and the panel screenshots exist.
#
# Usage:  bash run_fake_mission_e2e.sh OUTPUT_DIR [photo_info:1|0]
# Needs:  a sourced workspace with sim_demo + interfaces built (ROS 2 Humble), numpy, Pillow.
set -u
OUT="${1:-/tmp/sim_demo_e2e}"
PHOTO_INFO="${2:-1}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIX="$HERE/../fixtures/worlds"
rm -rf "$OUT"; mkdir -p "$OUT/run" "$OUT/shots"
# Photos live on the Linux file system (fast, like the real camera folder); results go to $OUT.
PHOTOS="$(mktemp -d /tmp/sim_demo_photos_XXXX)"
export QT_QPA_PLATFORM=offscreen
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}" GZ_PARTITION="${GZ_PARTITION:-dev_demo}"
COMMON=(-p world:=testfield -p "worlds_dir:=$FIX" -p "photo_dir:=$PHOTOS")
pids=()
start() {   # start <name> <command...>   (own process group, log in $OUT/<name>.log)
  local name="$1"; shift
  setsid "$@" >"$OUT/$name.log" 2>&1 &
  pids+=("$!")
}
cleanup() {
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
  sleep 3
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done
}
trap cleanup EXIT

start truth_detector timeout 140 ros2 run sim_demo truth_detector --ros-args "${COMMON[@]}"
start demo_markers   timeout 140 ros2 run sim_demo demo_markers   --ros-args "${COMMON[@]}"
start score_run      timeout 140 ros2 run sim_demo score_run      --ros-args "${COMMON[@]}" \
      -p "run_dir:=$OUT/run" -p linger_s:=4.0 -p gz_payload_tracking:=false
start demo_panel     timeout 140 ros2 run sim_demo demo_panel     --ros-args "${COMMON[@]}" \
      -p mission_link:=mavros -p "screenshot_path:=$OUT/shots/panel.png" \
      -p screenshot_after_s:=24.0 -p screenshot_every_s:=12.0 -p screenshot_count:=6
sleep 4
timeout 130 python3 "$HERE/fake_mission.py" --world testfield --worlds-dir "$FIX" \
    --photo-dir "$PHOTOS" --photo-info "$PHOTO_INFO" --summary "$OUT/summary.json" \
    >"$OUT/fake_mission.log" 2>&1
FAKE_EXIT=$?
sleep 6
# Keep a few photos in which the stand-in found targets (to look at), then remove the temp folder.
ls "$PHOTOS" > "$OUT/photo_names.txt"
n=0
grep -oE '[-0-9.]+ , [-0-9.]+\.jpg: (tent|person)[^[]*' "$OUT/truth_detector.log" | while read -r line; do
  name="${line%%: *}"
  n=$((n + 1)); [ "$n" -le 4 ] || break
  cp "$PHOTOS/$name" "$OUT/hit_${n}.jpg" 2>/dev/null && echo "$line" >> "$OUT/hits.txt"
done
rm -rf "$PHOTOS"
echo "== fake_mission exit: $FAKE_EXIT"; cat "$OUT/summary.json" 2>/dev/null
echo "== score report"; head -n 30 "$OUT/run/score_report.md" 2>/dev/null || echo "(missing)"
echo "== screenshots"; ls -la "$OUT/shots" 2>/dev/null
echo "== truth_detector (last lines)"; tail -n 8 "$OUT/truth_detector.log"
ok=1
[ "$FAKE_EXIT" -eq 0 ] || ok=0
grep -q 'Targets found:\*\* 2 of 2' "$OUT/run/score_report.md" 2>/dev/null || ok=0
[ -n "$(ls "$OUT/shots" 2>/dev/null)" ] || ok=0
echo "E2E_RESULT=$([ $ok -eq 1 ] && echo PASS || echo FAIL)"
