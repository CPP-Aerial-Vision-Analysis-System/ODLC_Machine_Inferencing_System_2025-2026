#!/usr/bin/env bash
# sim_worlds/scripts/fetch_models.sh -- PURPOSE: download, ahead of time, every Gazebo Fuel model the sim_worlds
# worlds use (the hiker mannequin "Rescue Randy", people, cars, trees, cones, ...), at the exact pinned versions
# the worlds reference, into Gazebo's Fuel cache (~/.gz/fuel, or $GZ_FUEL_CACHE_PATH).
# Why: without this, the FIRST start of a world downloads ~110 MB while the world loads and seems frozen; and
# offline, the missing objects are simply absent. Setup (simulation/setup) runs this once; it is safe to re-run
# (models already in the cache are skipped).
#
# usage:  fetch_models.sh            download whatever is missing
#         fetch_models.sh --check    only report what is missing (exit code 1 if something is missing)
#         fetch_models.sh --list     print the model URLs
# The list below must match sim_worlds/worldgen/assets.py (a unit test checks that).
set -u

SERVER="https://fuel.gazebosim.org/1.0"
MODELS=(
  "OpenRobotics/Rescue Randy/3"
  "OpenRobotics/Rescue Randy Sitting/8"
  "OpenRobotics/Standing person/3"
  "OpenRobotics/Walking person/3"
  "OpenRobotics/Casual female/4"
  "OpenRobotics/FemaleVisitor/2"
  "OpenRobotics/Hatchback blue/5"
  "OpenRobotics/SUV/4"
  "OpenRobotics/Pickup/4"
  "OpenRobotics/Oak tree/7"
  "OpenRobotics/Pine Tree/6"
  "OpenRobotics/Construction Cone/3"
  "OpenRobotics/Construction Barrel/4"
  "OpenRobotics/55gal_Drum/1"
  "OpenRobotics/foldable_chair/3"
  "OpenRobotics/foldable_table/4"
  "OpenRobotics/tent/4"
)
CACHE="${GZ_FUEL_CACHE_PATH:-$HOME/.gz/fuel}"
MODE="${1:-download}"

url_of()  { local owner="${1%%/*}"; local rest="${1#*/}"; echo "$SERVER/$owner/models/${rest%/*}/${rest##*/}"; }
dir_of()  { local owner="${1%%/*}"; local rest="${1#*/}"; local name="${rest%/*}"; local ver="${rest##*/}";
            echo "$CACHE/fuel.gazebosim.org/${owner,,}/models/${name,,}/$ver"; }
cached()  { [ -f "$(dir_of "$1")/model.config" ] || [ -f "$(dir_of "$1")/model.sdf" ]; }

case "$MODE" in
  --list)
    for m in "${MODELS[@]}"; do url_of "$m"; done; exit 0 ;;
  --check)
    missing=0
    for m in "${MODELS[@]}"; do
      if cached "$m"; then echo "  ok       $m"; else echo "  MISSING  $m"; missing=$((missing + 1)); fi
    done
    echo "fetch_models: ${#MODELS[@]} models, $missing missing (cache: $CACHE)"
    [ "$missing" -eq 0 ]; exit $? ;;
  download|--download) ;;
  -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
  *) echo "unknown option $MODE (use --check, --list or nothing)"; exit 2 ;;
esac

if ! command -v gz >/dev/null 2>&1; then
  echo "fetch_models: the 'gz' command is missing. Run: source /opt/ros/humble/setup.bash (Gazebo Harmonic installed?)"
  exit 2
fi
failed=0
for m in "${MODELS[@]}"; do
  if cached "$m"; then echo "  cached   $m"; continue; fi
  echo "  download $m ..."
  ok=0
  for attempt in 1 2 3; do                       # Fuel is sometimes slow: retry a few times
    if timeout 300 gz fuel download -u "$(url_of "$m")" >/dev/null 2>&1 && cached "$m"; then ok=1; break; fi
    sleep 3
  done
  if [ "$ok" -eq 1 ]; then echo "           done"; else echo "           FAILED (offline? try again later)"; failed=$((failed + 1)); fi
done
echo "fetch_models: ${#MODELS[@]} models, $failed failed (cache: $CACHE)"
[ "$failed" -eq 0 ]
