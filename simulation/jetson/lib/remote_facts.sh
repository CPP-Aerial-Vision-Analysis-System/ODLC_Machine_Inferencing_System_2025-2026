#!/bin/bash
# =============================================================================
#  simulation/jetson/lib/remote_facts.sh  --  read-only facts about a Jetson, as KEY=VALUE lines
#
#  PURPOSE: jetson.sh check (and deploy/start/warmup before they do anything) sends this script over the owner's SSH
#  login and runs it ON the Jetson with  bash -s . It only LOOKS: it never installs, writes, starts or stops anything
#  and never asks for a password (sudo is only tried as "sudo -n", which fails at once when a password would be
#  needed). It prints one KEY=VALUE line per fact (values on one line), for lib/jetson_facts.py to judge on the PC:
#  the board and its JetPack/L4T release, Docker and the NVIDIA runtime, disk, memory, whether the team is using the
#  board right now ("team busy?", HIL_DESIGN.md 9.6), the power mode and its list, the clocks, the ROS domains in use,
#  our image/container/engine, and the Jetson's clock. The virtual Jetson runs it on this PC the same way.
#
#  Usage (normally only through jetson.sh):  bash remote_facts.sh [--only all|busy] [--data DIR] [--image NAME]
#                                                                  [--container NAME] [--team-root DIR]
#  Test hook: FACTS_ROOT=/some/folder makes it read /etc, /proc/device-tree and /sys files below that folder.
# =============================================================================
ONLY=all; DATA="$HOME/astra_hil"; IMAGE="astra-hil:jetson"; CONTAINER="astra-hil"; TEAM_ROOT="$HOME/astra"
while [ $# -gt 0 ]; do
  case "$1" in
    --only) ONLY="$2"; shift ;;
    --data) DATA="$2"; shift ;;
    --image) IMAGE="$2"; shift ;;
    --container) CONTAINER="$2"; shift ;;
    --team-root) TEAM_ROOT="$2"; shift ;;
  esac
  shift
done
R="${FACTS_ROOT:-}"
PROC="${FACTS_PROC:-/proc}"          # test hook: a fake process table (folders named by pid, with cmdline/cgroup/environ)
LABEL="org.astra.owner=hil"

kv() { printf '%s=%s\n' "$1" "$(printf '%s' "$2" | tr '\n\r\0' '   ' | sed 's/[[:space:]]*$//')"; }
have() { command -v "$1" >/dev/null 2>&1; }

kv FACTS_VERSION 1
kv FACTS_TIME_UNIX "$(date +%s.%N 2>/dev/null)"
kv USER "$(id -un 2>/dev/null)"
kv UID "$(id -u 2>/dev/null)"
kv GID "$(id -g 2>/dev/null)"
kv HOME "$HOME"
kv DATA_DIR "$DATA"

# ---- docker access: direct (docker group) / sudo -n (passwordless sudo) / none ----------------------------------
DK=""
if have docker; then
  kv DOCKER_PRESENT yes
  if docker version --format '{{.Server.Version}}' >/dev/null 2>&1; then DK="docker"; kv DOCKER_ACCESS direct
  elif sudo -n docker version --format '{{.Server.Version}}' >/dev/null 2>&1; then DK="sudo -n docker"; kv DOCKER_ACCESS sudo-n
  else
    kv DOCKER_ACCESS none
    kv DOCKER_ERROR "$(docker version --format '{{.Server.Version}}' 2>&1 | tail -1)"
  fi
else
  kv DOCKER_PRESENT no; kv DOCKER_ACCESS none
fi
dk() { [ -n "$DK" ] && $DK "$@"; }

# ---- team busy? (HIL_DESIGN.md 9.6): team programs not inside our containers, GPU load, other containers ---------
our_ids=""
if [ -n "$DK" ]; then our_ids=$(dk ps -q --no-trunc --filter "label=$LABEL" 2>/dev/null | tr '\n' ' '); fi
procs=""
pat='mavros_node|ros2 launch|tracker\.launch|new_od|main_controller|/siyi|gpstest|waypoint|trtexec|detector_node|zr10_node|stream_node|live_node'
for d in "$PROC"/[0-9]*; do
  p="${d##*/}"
  [ "$p" = "$$" ] && continue
  cmd=$( { tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null ) || continue
  [ -n "$cmd" ] || continue
  case "$cmd" in *remote_facts*|*"bash -s"*) continue ;; esac
  printf '%s' "$cmd" | grep -Eq "$pat" || continue
  mine=0
  if [ -n "$our_ids" ]; then
    cg=$(cat "$d/cgroup" 2>/dev/null)
    for id in $our_ids; do case "$cg" in *"$id"*) mine=1 ;; esac; done
  fi
  [ "$mine" = 1 ] && continue
  u=$(stat -c %U "$d" 2>/dev/null)
  name=$(printf '%s' "$cmd" | grep -Eo "$pat" | head -1 | sed 's#^/##')
  procs="$procs${procs:+;}$name pid $p user $u"
done
kv TEAM_PROCS "$procs"

# GPU load: tegrastats GR3D_FREQ (3 samples, 1 s apart), else the sysfs load (0-1000)
gpu=""
if have tegrastats; then
  gpu=$(timeout 4 tegrastats --interval 1000 2>/dev/null | head -3 | grep -Eo 'GR3D_FREQ [0-9]+%' | grep -Eo '[0-9]+' \
        | awk '{s+=$1; n++} END {if (n) printf "%.0f", s/n}')
  kv TEGRASTATS_PRESENT yes
else
  kv TEGRASTATS_PRESENT no
fi
if [ -z "$gpu" ]; then
  for f in "$R"/sys/devices/platform/gpu.0/load "$R"/sys/devices/gpu.0/load; do
    if [ -r "$f" ]; then
      gpu=$( (cat "$f"; sleep 1; cat "$f"; sleep 1; cat "$f") 2>/dev/null | awk '{s+=$1; n++} END {if (n) printf "%.0f", s/n/10}')
      break
    fi
  done
fi
kv GPU_LOAD_PCT "$gpu"

# other running containers (not ours) and their CPU use
others=""
if [ -n "$DK" ]; then
  while IFS='|' read -r name cpu; do
    [ -n "$name" ] || continue
    others="$others${others:+;}$name $cpu"
  done < <(dk stats --no-stream --format '{{.Name}}|{{.CPUPerc}}' 2>/dev/null | while IFS='|' read -r n c; do
             lab=$(dk inspect -f '{{index .Config.Labels "org.astra.owner"}}' "$n" 2>/dev/null)
             [ "$lab" = hil ] || printf '%s|%s\n' "$n" "$c"
           done)
fi
kv OTHER_CONTAINERS "$others"
mi="$R/proc/meminfo"; [ -r "$mi" ] || mi=/proc/meminfo
kv MEM_TOTAL_MB "$(awk '/^MemTotal:/{printf "%d", $2/1024}' "$mi" 2>/dev/null)"
kv MEM_AVAIL_MB "$(awk '/^MemAvailable:/{printf "%d", $2/1024}' "$mi" 2>/dev/null)"
kv WHO "$(who 2>/dev/null | awk '{print $1"@"$NF}' | sort | uniq -c | awk '{printf "%s%s x%s", (NR>1?";":""), $2, $1}')"

[ "$ONLY" = busy ] && exit 0

# ---- the board and its release ------------------------------------------------------------------------------------
kv UNAME_M "$(uname -m)"
kv KERNEL "$(uname -r)"
kv HOSTNAME "$(hostname 2>/dev/null)"
kv MODEL "$( { tr -d '\0' < "$R/proc/device-tree/model"; } 2>/dev/null)"
rel=$(head -1 "$R/etc/nv_tegra_release" 2>/dev/null)
kv NV_TEGRA_RELEASE "$rel"
if [ -n "$rel" ]; then
  maj=$(printf '%s' "$rel" | sed -n 's/^# R\([0-9]*\) .*/\1/p')
  rev=$(printf '%s' "$rel" | sed -n 's/.*REVISION: \([0-9.]*\),.*/\1/p')
  kv L4T "R${maj}.${rev}"
else
  kv L4T ""
fi
if [ -r "$R/etc/os-release" ]; then
  kv OS_ID "$(. "$R/etc/os-release"; printf '%s' "$ID")"
  kv OS_VERSION_ID "$(. "$R/etc/os-release"; printf '%s' "$VERSION_ID")"
fi
kv JETPACK_PKG "$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)"
cuda=$(grep -A3 '"cuda"' "$R/usr/local/cuda/version.json" 2>/dev/null | sed -n 's/.*"version" *: *"\([^"]*\)".*/\1/p' | head -1)
[ -n "$cuda" ] || cuda=$(dpkg-query -W -f='${Version}' cuda-cudart-12-6 2>/dev/null)
kv CUDA_VERSION "$cuda"
kv TENSORRT_VERSION "$(dpkg-query -W -f='${Version}' tensorrt 2>/dev/null || dpkg-query -W -f='${Version}' libnvinfer10 2>/dev/null)"
kv GRAPHICAL "$( (systemctl is-active gdm 2>/dev/null || systemctl is-active display-manager 2>/dev/null) | head -1)"

# ---- docker details -----------------------------------------------------------------------------------------------
if [ -n "$DK" ]; then
  kv DOCKER_VERSION "$(dk version --format '{{.Server.Version}}' 2>/dev/null)"
  kv DOCKER_RUNTIMES "$(dk info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null)"
  kv DOCKER_DEFAULT_RUNTIME "$(dk info --format '{{.DefaultRuntime}}' 2>/dev/null)"
  root=$(dk info --format '{{.DockerRootDir}}' 2>/dev/null); kv DOCKER_ROOT "$root"
fi
kv NVIDIA_CTK_VERSION "$(nvidia-ctk --version 2>/dev/null | head -1)"
root="${root:-/var/lib/docker}"
dfline=$(df -BG "$root" 2>/dev/null | awk 'NR==2' ); [ -n "$dfline" ] || dfline=$(df -BG / | awk 'NR==2')
kv DISK_DOCKER_FREE_GB "$(printf '%s' "$dfline" | awk '{gsub("G","",$4); print $4}')"
kv DISK_DOCKER_TOTAL_GB "$(printf '%s' "$dfline" | awk '{gsub("G","",$2); print $2}')"
kv DISK_HOME_FREE_GB "$(df -BG "$HOME" 2>/dev/null | awk 'NR==2 {gsub("G","",$4); print $4}')"

# ---- power mode and clocks (report only; HIL_DESIGN.md 9.2 row 13) ------------------------------------------------
q=""
if have nvpmodel; then
  q=$(sudo -n nvpmodel -q 2>/dev/null) || q=$(nvpmodel -q 2>/dev/null)
fi
kv NVPMODEL_MODE "$(printf '%s\n' "$q" | sed -n 's/^NV Power Mode: *//p' | head -1)"
kv NVPMODEL_ID "$(printf '%s\n' "$q" | grep -E '^[0-9]+$' | head -1)"
kv NVPMODEL_LIST "$(grep -Eo 'POWER_MODEL ID=[0-9]+ NAME=[A-Za-z0-9_]+' "$R/etc/nvpmodel.conf" 2>/dev/null \
                   | sed 's/POWER_MODEL ID=\([0-9]*\) NAME=\(.*\)/\1=\2/' | paste -sd, -)"
cmin=$(cat "$R/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq" 2>/dev/null)
cmax=$(cat "$R/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq" 2>/dev/null)
gdir=$(ls -d "$R"/sys/class/devfreq/*gpu* 2>/dev/null | head -1)
gmin=""; gmax=""
[ -n "$gdir" ] && { gmin=$(cat "$gdir/min_freq" 2>/dev/null); gmax=$(cat "$gdir/max_freq" 2>/dev/null); }
if [ -n "$cmin" ] && [ -n "$cmax" ] && [ -n "$gmin" ] && [ -n "$gmax" ]; then
  if [ "$cmin" = "$cmax" ] && [ "$gmin" = "$gmax" ]; then kv CLOCKS locked; else kv CLOCKS not-locked; fi
else
  kv CLOCKS unknown
fi

# ---- ROS domains the board's ROS processes use (only readable for our own user's processes) -----------------------
doms=""
for d in "$PROC"/[0-9]*; do
  c=$( { tr '\0' ' ' < "$d/cmdline"; } 2>/dev/null ) || continue
  case "$c" in *ros2*|*_node*|*launch*) ;; *) continue ;; esac
  env=$( { tr '\0' '\n' < "$d/environ"; } 2>/dev/null ) || continue
  d=$(printf '%s\n' "$env" | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)
  doms="$doms ${d:-0}"
done
kv ROS_DOMAINS "$(printf '%s\n' $doms | sort -un | paste -sd, -)"

# ---- ports, sshd forwarding settings (best effort; check row 12 tests the real thing) -----------------------------
if have ss; then
  if ss -ltn 2>/dev/null | awk '{print $4}' | grep -Eq '[:.]29517$'; then kv PORT_29517 busy; else kv PORT_29517 free; fi
fi
kv SSHD_FORWARDING "$(grep -Ei '^[[:space:]]*(AllowTcpForwarding|AllowStreamLocalForwarding|DisableForwarding)[[:space:]]' "$R/etc/ssh/sshd_config" "$R"/etc/ssh/sshd_config.d/*.conf 2>/dev/null | sed 's/^[^:]*://' | tr -s ' \t' ' ' | paste -sd, -)"

# ---- our things: data folder, image, container, engine ------------------------------------------------------------
[ -d "$DATA" ] && kv DATA_EXISTS yes || kv DATA_EXISTS no
if [ -n "$DK" ]; then
  kv HIL_IMAGE_ID "$(dk image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null)"
  kv HIL_IMAGE_SHA "$(dk image inspect -f '{{index .Config.Labels "org.astra.git_sha"}}' "$IMAGE" 2>/dev/null)"
  kv HIL_IMAGE_BUILT "$(dk image inspect -f '{{index .Config.Labels "org.astra.built"}}' "$IMAGE" 2>/dev/null)"
  kv HIL_IMAGE_CODE "$(dk image inspect -f '{{index .Config.Labels "org.astra.code_sha"}}' "$IMAGE" 2>/dev/null)"
  kv HIL_IMAGE_TRT "$(dk image inspect -f '{{index .Config.Labels "org.astra.tensorrt"}}' "$IMAGE" 2>/dev/null)"
  kv HIL_CONTAINER "$(dk inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null)"
  kv HIL_IMAGES_GB "$(dk images --filter "label=$LABEL" --format '{{.Size}}' 2>/dev/null | paste -sd, -)"
fi
if [ -r "$DATA/deployed.env" ]; then
  while IFS='=' read -r k v; do case "$k" in [A-Z]*) kv "DEPLOYED_$k" "$v" ;; esac; done < "$DATA/deployed.env"
fi
if [ -r "$DATA/models/yolo26m.engine.meta" ]; then
  while IFS='=' read -r k v; do case "$k" in [A-Z]*) kv "ENGINE_$k" "$v" ;; esac; done < "$DATA/models/yolo26m.engine.meta"
  kv ENGINE_PRESENT "$([ -s "$DATA/models/yolo26m.engine" ] && echo yes || echo no)"
else
  kv ENGINE_PRESENT no
fi
kv MODEL_PRESENT "$([ -s "$DATA/models/yolo26m.pt" ] && echo yes || echo no)"
# the GPU's name as the driver reports it (best effort; the virtual Jetson's engine must be for THIS PC's GPU)
kv GPU_NAME "$( (nvidia-smi --query-gpu=name --format=csv,noheader || /usr/lib/wsl/lib/nvidia-smi --query-gpu=name --format=csv,noheader) 2>/dev/null | head -1)"
kv TEAM_ROOT "$TEAM_ROOT"
kv TEAM_MODEL "$([ -s "$TEAM_ROOT/ros2_ws/src/yolo26m.pt" ] && echo yes || echo no)"
kv CLOCK_UNIX "$(date +%s.%N 2>/dev/null)"
