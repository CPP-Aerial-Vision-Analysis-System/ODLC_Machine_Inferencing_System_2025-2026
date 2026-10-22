#!/usr/bin/env bash
# =============================================================================
#  compose_up.sh  --  THE FULL SEARCH DEMO IN DOCKER: simulator + Jetson stand-in, one command
#
#  PURPOSE: starts docker/compose.sim.yml the right way for this computer and shows what happens:
#    * "sim" container (192.168.144.25 = the real camera's address): Gazebo, ArduCopter SITL, MAVProxy, the SIYI
#      camera emulator, RViz, the demo panel and the score recorder (simulation/docker/container_sim.sh);
#    * "jetson" container (192.168.144.100 = the real Jetson's address): the team's ./launcher.sh sim, unchanged
#      (simulation/docker/container_jetson.sh).
#  By default the sim container then flies the demo by itself like simulation/scripts/demo.sh: waits for the
#  autopilot, uploads the survey mission, waits until the team's code in the jetson container is ready, arms,
#  AUTO, search, detections, drops, home, landing, score report. Ctrl+C stops everything cleanly.
#  Before starting it: checks Docker and the images, picks the repo folder (on Windows the Linux-side copy),
#  makes the data folder (~/.astra_docker: runs, logs, detector weights), takes the native simulator's camera
#  address off this computer (it would steal the sim container's traffic), and chooses the extra compose
#  files: compose.gpu.yml (CUDA, when the NVIDIA Container Toolkit is there), compose.linux.yml (Linux PC).
#  Afterwards: saves both containers' logs into the run folder and removes the containers (builds are kept).
#
#  HOW TO RUN (inside the Ubuntu-22.04 WSL distro, or on Linux; images built once with build_images.sh):
#      bash simulation/docker/compose_up.sh                    runway world, full demo, windows on the desktop
#      bash simulation/docker/compose_up.sh suas_field         the SUAS 2026 competition field
#      bash simulation/docker/compose_up.sh runway --detector truth --camera a8mini
#  Options:  --detector real|truth   who finds the targets (real = the team's new_od in the jetson container)
#            --camera zr10|a8mini    photo profile (zr10 = 2K photos, default; a8mini = 4K)
#            --headless              no windows (automatic when there is no desktop)
#            --manual                no automatic flight: fly with the demo panel's buttons or Mission Planner
#            --keep                  keep everything running after the landing (stop: compose_down.sh)
#            --timeout MIN           give the demo up after MIN minutes (default 30)
#            --no-gpu                no CUDA for the containers (the detector then runs on the processor)
#            --no-windows-gcs        no MAVLink copy for Mission Planner / QGroundControl on Windows
#            --windows-gcs HOST[:PORT]  send that copy somewhere else (default: the Windows host, port 14550)
#            --only sim              start only the simulator (your own Jetson stand-in: run_jetson_container.sh --net astra)
#            --repo DIR              mount this repo folder (default: the Linux-side copy on Windows, else this repo)
#            --sync                  first copy the Windows repo to the Linux-side copy (after editing on Windows)
#            --sync-only             only do that copy, start nothing
#            --rebuild               clean colcon build of ros2_ws inside both containers
#            --build-jobs N          colcon jobs per container (default 2)
#            --detach                run in the background (watch: docker compose -p astra logs -f)
#            --force                 start even if a native simulator is running
#            --dry-run               check and print everything, start nothing
#  Results:  ~/.astra_docker/runs/<time>_<world>/  (score_report.md, photos, logs); on Windows also at
#            \\wsl.localhost\Ubuntu-22.04\home\<you>\.astra_docker\runs
#  Stop:     Ctrl+C here, or (with --detach / --keep)  bash simulation/docker/compose_down.sh
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

WORLD=runway; DETECTOR=real; CAMERA=zr10; HEADLESS=0; DEMO=1; KEEP=0; TIMEOUT_MIN=30; GPU=auto
WIN_GCS=auto; ONLY=""; REPO_ARG=""; SYNC=0; REBUILD=0; BUILD_JOBS=2; DETACH=0; FORCE=0; FIRST=1
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --detector) shift; DETECTOR="${1:-}" ;;
    --camera) shift; CAMERA="${1:-}" ;;
    --headless) HEADLESS=1 ;;
    --manual) DEMO=0 ;;
    --keep) KEEP=1 ;;
    --timeout) shift; TIMEOUT_MIN="${1:-}" ;;
    --no-gpu) GPU=no ;;
    --gpu) GPU=yes ;;
    --no-windows-gcs) WIN_GCS="" ;;
    --windows-gcs) shift; WIN_GCS="${1:-}"; [ -n "$WIN_GCS" ] || die "--windows-gcs needs HOST or HOST:PORT" ;;
    --only) shift; ONLY="${1:-}"; [ "$ONLY" = sim ] || die "--only takes: sim" ;;
    --repo) shift; REPO_ARG="${1:-}" ;;
    --sync) SYNC=1 ;;
    --sync-only) SYNC=2 ;;
    --rebuild) REBUILD=1 ;;
    --build-jobs) shift; BUILD_JOBS="${1:-}" ;;
    --detach|-d) DETACH=1 ;;
    --force) FORCE=1 ;;
    --dry-run) ASTRA_DRY=1 ;;
    -*) die "unknown option $1 (see --help)" ;;
    *) if [ "$FIRST" = 1 ]; then WORLD="$1"; FIRST=0; else die "unexpected argument \"$1\" (see --help)"; fi ;;
  esac
  shift
done
case "$DETECTOR" in real|truth) ;; *) die "--detector must be real or truth" ;; esac
case "$CAMERA" in zr10|a8mini) ;; *) die "--camera must be zr10 or a8mini" ;; esac
for n in "$TIMEOUT_MIN" "$BUILD_JOBS"; do case "$n" in ''|*[!0-9]*) die "\"$n\" is not a whole number" ;; esac; done
[ -n "$WIN_GCS" ] && [ "$WIN_GCS" != auto ] && case "$WIN_GCS" in *:*) ;; *) WIN_GCS="$WIN_GCS:14550" ;; esac
if [ "$SYNC" = 2 ]; then astra_sync_copy; exit $?; fi

say "ASTRA simulator in Docker: \"sim\" ($ASTRA_CAMERA_IP) + \"jetson\" ($ASTRA_JETSON_IP)"

# ---- 1. Docker + images ----------------------------------------------------------------------------
astra_need_docker
if [ "$ONLY" = sim ]; then
  astra_require_images "astra-sim:latest sim"
else
  astra_require_images "astra-sim:latest sim" "astra-dev:latest dev"
fi

# ---- 2. repo folder + data folder --------------------------------------------------------------------
[ "$SYNC" = 1 ] && astra_sync_copy
REPO_HOST="$(astra_pick_repo "$REPO_ARG")" || exit 1
export ASTRA_REPO_HOST="$REPO_HOST"
[ -f "$REPO_HOST/simulation/docker/container_sim.sh" ] \
  || die "$REPO_HOST has no simulation/docker/container_sim.sh: the copy is older than these scripts. Run with --sync."
astra_prepare_data
astra_use_host_fuel

# The world: a name (runway, suas_field) or a file INSIDE the mounted repo (the container only sees the repo).
case "$WORLD" in
  runway|suas_field|standin) CWORLD="$WORLD" ;;
  *.sdf|*.yaml|*.yml)
    f="$(readlink -f "$WORLD" 2>/dev/null || true)"
    case "$f" in
      "$REPO_HOST"/*) CWORLD="$ASTRA_CONTAINER_REPO/${f#"$REPO_HOST"/}" ;;
      *) die "world file $WORLD must be inside the mounted repo ($REPO_HOST) so the container can see it" ;;
    esac ;;
  *) CWORLD="$WORLD"; warn "\"$WORLD\" is not runway / suas_field; the simulator will look for sim_worlds/config/$WORLD.yaml" ;;
esac

# ---- 3. things on this computer that would get in the way ---------------------------------------------
if [ "$ASTRA_DRY" = 0 ] && docker ps -q --filter 'name=^astra-(sim|jetson)$' | grep -q .; then
  warn "The containers of an earlier start are still running: stopping them first"
  bash "$ASTRA_DOCKER_SCRIPTS/compose_down.sh" --quiet || true
fi
if astra_native_sim_running; then
  if [ "$FORCE" = 1 ]; then
    warn "a NATIVE simulator (Gazebo / SITL) is running too (--force): expect a slow laptop"
  else
    die "A NATIVE simulator (Gazebo / SITL outside Docker) is running. Two simulators at once make the laptop
    crawl. Stop it first:  bash $ASTRA_REPO_HERE/simulation/scripts/stop_sim.sh   (or add --force)"
  fi
fi
astra_free_camera_ip

# ---- 4. which compose files (display, GPU) -----------------------------------------------------------
FILES=(-f "$ASTRA_REPO_HERE/docker/compose.sim.yml")
DISPLAY_MODE="$(astra_display_mode)"
[ "$HEADLESS" = 1 ] && DISPLAY_MODE=none
if [ "$GPU" != no ] && { [ "$ASTRA_DRY" = 1 ] || astra_has_nvidia_runtime; }; then
  FILES+=(-f "$ASTRA_REPO_HERE/docker/compose.gpu.yml")
  GPU_TEXT="NVIDIA card for CUDA (compose.gpu.yml)"
else
  [ "$GPU" = yes ] && die "--gpu: Docker has no NVIDIA runtime (install_docker_engine.sh --nvidia)"
  GPU_TEXT="no CUDA (the detector runs on the processor: slower)"
fi
if ! astra_is_wsl; then
  # A Linux PC: X11 + its own graphics instead of WSLg/DirectX. Last file, so its settings win.
  FILES+=(-f "$ASTRA_REPO_HERE/docker/compose.linux.yml")
  [ -e /dev/dri ] || die "this Linux PC has no /dev/dri (graphics device); delete the devices lines in docker/compose.linux.yml"
  [ "$DISPLAY_MODE" = x11 ] && astra_x11_allow
fi
SIM_OPTIONS=""; LINGER=120
if [ "$DISPLAY_MODE" = none ]; then SIM_OPTIONS="--headless"; LINGER=0; fi

# ---- 5. Mission Planner on Windows -----------------------------------------------------------------
[ "$WIN_GCS" = auto ] && WIN_GCS="$(astra_windows_gcs)"

# ---- 6. settings for compose (docker/compose.sim.yml reads these) -----------------------------------
export ASTRA_SIM_WORLD="$CWORLD" ASTRA_CAMERA="$CAMERA" ASTRA_DETECTOR="$DETECTOR" ASTRA_SIM_OPTIONS="$SIM_OPTIONS"
export ASTRA_DEMO="$DEMO" ASTRA_KEEP="$KEEP" ASTRA_TIMEOUT_MIN="$TIMEOUT_MIN" ASTRA_LINGER_S="$LINGER"
export ASTRA_WINDOWS_GCS="$WIN_GCS" ASTRA_BUILD_JOBS="$BUILD_JOBS"
export ASTRA_BUILD="${ASTRA_BUILD:-auto}"
[ "$REBUILD" = 1 ] && export ASTRA_BUILD=clean
COMPOSE=(docker compose -p astra "${FILES[@]}")
# --only sim: no jetson container; the automatic demo then waits for YOUR Jetson stand-in to get ready
# (run_jetson_container.sh --net astra --launcher), like demo.sh waits for ./launcher.sh sim.
SERVICES=(); [ "$ONLY" = sim ] && SERVICES=(sim)

info "world      $CWORLD"
info "detector   $DETECTOR | camera $CAMERA | $([ "$DEMO" = 1 ] && echo "automatic demo flight" || echo "manual: fly from the demo panel / Mission Planner")"
info "repo       $REPO_HOST  ->  $ASTRA_CONTAINER_REPO in both containers"
info "data       $ASTRA_DOCKER_DATA  (runs, logs, detector weights)"
info "windows    $DISPLAY_MODE$([ "$DISPLAY_MODE" = none ] && echo " (headless)")"
info "graphics   $GPU_TEXT"
info "Windows GS ${WIN_GCS:-none}$([ -n "$WIN_GCS" ] && echo " (Mission Planner: connection type UDP, port ${WIN_GCS##*:})")"
info "compose    ${COMPOSE[*]}"
astra_mem_check 4000

if [ "$ASTRA_DRY" = 1 ]; then
  say "DRY RUN: checking the compose files with these settings (nothing is started)"
  if "${COMPOSE[@]}" config --quiet; then ok "docker compose config: valid"; else die "docker compose config found a problem (above)"; fi
  if [ "$DETACH" = 1 ]; then astra_run "${COMPOSE[@]}" up -d --no-build "${SERVICES[@]}"
  else astra_run "${COMPOSE[@]}" up --no-build --abort-on-container-exit --exit-code-from sim "${SERVICES[@]}"; fi
  exit 0
fi

# ---- 7. start ---------------------------------------------------------------------------------------
if [ "$DETACH" = 1 ]; then
  "${COMPOSE[@]}" up -d --no-build "${SERVICES[@]}" || die "docker compose up failed (see above)"
  ok "started in the background"
  info "watch:   docker compose -p astra logs -f        (or: docker logs -f astra-sim / astra-jetson)"
  info "stop:    bash $ASTRA_DOCKER_SCRIPTS/compose_down.sh"
  exit 0
fi
say "Starting (first start: each container builds ros2_ws, a few minutes). Ctrl+C stops everything."
info "Lines start with the container name: \"astra-sim\" = simulator + demo steps, \"astra-jetson\" = the team's code."
trap 'echo; info "Ctrl+C: stopping the containers cleanly (the score report is still written) ..."' INT
"${COMPOSE[@]}" up --no-build --abort-on-container-exit --exit-code-from sim "${SERVICES[@]}"
RC=$?
trap - INT

# ---- 8. afterwards: logs into the run folder, remove the containers ----------------------------------
RUN_DIR="$(cat "$ASTRA_DOCKER_DATA/state/docker_run_dir" 2>/dev/null || true)"
if [ -n "$RUN_DIR" ]; then
  # The path inside the container (/home/ubuntu/.astra_sim/...) is the data folder on this computer.
  RUN_DIR="$ASTRA_DOCKER_DATA/${RUN_DIR#/home/ubuntu/.astra_sim/}"
  if [ -d "$RUN_DIR" ]; then
    docker logs astra-sim > "$RUN_DIR/container_sim.log" 2>&1 || true
    docker logs astra-jetson > "$RUN_DIR/container_jetson.log" 2>&1 || true
  fi
fi
"${COMPOSE[@]}" down >/dev/null 2>&1 || true
echo
if [ -n "$RUN_DIR" ] && [ -d "$RUN_DIR" ]; then
  say "Everything from this run: $RUN_DIR"
  [ -s "$RUN_DIR/score_report.md" ] && info "score report: $RUN_DIR/score_report.md"
  astra_is_wsl && info "in Windows Explorer: \\\\wsl.localhost\\${WSL_DISTRO_NAME:-Ubuntu-22.04}$(printf '%s' "$RUN_DIR" | tr '/' '\\')"
fi
[ "$RC" = 0 ] && ok "finished" || warn "the simulator container ended with code $RC (its log: $RUN_DIR/container_sim.log)"
exit "$RC"
