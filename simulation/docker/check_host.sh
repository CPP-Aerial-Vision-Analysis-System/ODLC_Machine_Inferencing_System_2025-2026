#!/usr/bin/env bash
# =============================================================================
#  check_host.sh  --  the Docker "doctor": is THIS computer ready for the ASTRA containers? (changes nothing)
#
#  PURPOSE: a teammate's first Docker run can fail for many reasons that have nothing to do with our code:
#  Docker is not installed or not running, "docker" is the wrong program (Docker Desktop's Windows helper), the
#  user may not use Docker yet, the build tool (buildx) is missing, the disk is full, the clock is wrong, a school
#  network blocks a download, the drone network 192.168.144.0/24 is already taken, windows cannot open, the
#  graphics card is not reachable, scripts got Windows line endings (CRLF) from a Windows git clone, an image is
#  missing, older than docker/Dockerfile or built for another user, a native simulator is still running ...
#  This script looks at each of these in the order a first run meets them, and for every problem prints what is
#  wrong and EXACTLY what to type to fix it. It installs, deletes and starts nothing (only --fix-line-endings
#  changes files, and only their invisible line ends; --deep starts small test containers).
#
#  HOW TO RUN (inside Ubuntu: the WSL distro on Windows, or a Linux PC; from any folder):
#      bash simulation/docker/check_host.sh                     all checks (about 30 s, up to a minute for a
#                                                               checkout on a Windows drive; no images needed)
#      bash simulation/docker/check_host.sh --offline           without the internet checks
#      bash simulation/docker/check_host.sh --deep              also start test containers: the images' own
#                                                               self-test (build_images.sh --check) and the GPU
#      bash simulation/docker/check_host.sh --fix-line-endings  turn Windows line endings (CRLF) back into Linux
#                                                               ones (LF) in the files git keeps with LF (your
#                                                               edits stay; Windows .bat/.cmd/.ps1 keep CRLF)
#      bash simulation/docker/check_host.sh --repo DIR          check that checkout instead of this one
#  Exit code: 0 = nothing blocks Docker here (warnings may remain), 1 = at least one [FAIL] to fix first.
#  Windows: the Windows side (Docker Desktop, WSL, git settings, memory) is checked by check_host.ps1:
#      powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1
#  Every message is explained in docs/simulation/DOCKER.md, "Troubleshooting".
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

OFFLINE=0; DEEP=0; FIX_EOL=0; REPO_ARG=""
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --offline) OFFLINE=1 ;;
    --deep) DEEP=1 ;;
    --fix-line-endings) FIX_EOL=1 ;;
    --repo) shift; REPO_ARG="${1:-}"; [ -n "$REPO_ARG" ] || die "--repo needs a folder" ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done
[ "${ASTRA_CHECK_OFFLINE:-0}" = 1 ] && OFFLINE=1      # the tests use this

# ---- report helpers: one line per finding, the fix on the lines below it ----------------------------------
FAILS=0; WARNS=0
pass()    { printf '  [ OK ]  %s\n' "$*"; }
note()    { printf '  [ .. ]  %s\n' "$*"; }
caution() { printf '  [WARN]  %s\n' "$*"; WARNS=$((WARNS + 1)); }
fail()    { printf '  [FAIL]  %s\n' "$*"; FAILS=$((FAILS + 1)); }
tip()     { printf '          -> %s\n' "$*"; }
title()   { printf '\n%s\n' "$*"; }

# Which checkout: --repo DIR, else the repo these scripts belong to. The containers mount the Linux-side copy
# instead when the checkout is on a Windows drive and the copy exists (the same choice as astra_pick_repo).
if [ -n "$REPO_ARG" ]; then
  GIT_REPO="$(cd "$REPO_ARG" 2>/dev/null && pwd)" || die "--repo $REPO_ARG: no such folder"
  MOUNT_REPO="$GIT_REPO"
else
  GIT_REPO="$ASTRA_REPO_HERE"; MOUNT_REPO="$ASTRA_REPO_HERE"
  case "$GIT_REPO" in /mnt/*) [ -d "$ASTRA_LINUX_COPY/ros2_ws/src" ] && MOUNT_REPO="$ASTRA_LINUX_COPY" ;; esac
fi

# fix_line_endings REPO : rewrite CRLF -> LF in every file git stores with LF but finds with CRLF on disk (what a
# Windows clone with core.autocrlf=true does). Content is not changed, so git then sees the files as unchanged.
fix_line_endings() {
  local repo="$1" rec meta f n=0 kept=0 changed=() real=() same=() edited=0
  command -v git >/dev/null 2>&1 || die "--fix-line-endings needs git:  sudo apt-get install git"
  git -C "$repo" rev-parse --git-dir >/dev/null 2>&1 \
    || die "$repo is not a git checkout (a ZIP download?). Repair a file by hand instead:  sed -i 's/\\r\$//' FILE"
  say "Turning Windows line endings (CRLF) back into Linux ones (LF) in $repo"
  info "only files git keeps with LF; Windows launchers (.bat, .cmd, .ps1) keep CRLF; your edits stay as they are"
  case "$repo" in /mnt/*) info "(git reads every file through the Windows drive: this takes about a minute)" ;; esac
  while IFS= read -r -d '' rec; do
    meta="${rec%%$'\t'*}"; f="${rec#*$'\t'}"
    case "$meta" in "i/lf "*"w/crlf"*|"i/lf "*"w/mixed"*) ;; *) continue ;; esac
    case "$meta" in *"eol=crlf"*) kept=$((kept + 1)); continue ;; esac
    { [ -f "$repo/$f" ] && [ ! -L "$repo/$f" ]; } || continue
    if sed -i 's/\r$//' "$repo/$f"; then n=$((n + 1)); changed+=("$f"); info "LF now: $f"; fi
  done < <(git -C "$repo" ls-files --eol -z 2>/dev/null)
  ok "$n file(s) changed to LF$([ "$kept" -gt 0 ] && echo "; $kept Windows file(s) keep CRLF on purpose")"
  # git trusts the file SIZE it noted at checkout: a file that only lost its "\r"s would still be listed as
  # "modified" (with an empty diff). Re-adding exactly the files whose content now equals git's copy updates that
  # note and stages nothing new. Files that also carry your own edits are left alone (they stay "modified").
  if [ "${#changed[@]}" -gt 0 ]; then
    while IFS= read -r -d '' f; do real+=("$f"); done < <(git -C "$repo" diff --name-only -z -- "${changed[@]}" 2>/dev/null)
    for f in "${changed[@]}"; do
      case " ${real[*]} " in *" $f "*) edited=$((edited + 1)) ;; *) same+=("$f") ;; esac
    done
    [ "${#same[@]}" -gt 0 ] && git -C "$repo" add -- "${same[@]}" 2>/dev/null
    info "git now sees ${#same[@]} of them as unchanged$([ "$edited" -gt 0 ] && echo "; $edited contain your own edits and stay \"modified\", as before")"
  fi
  # The Linux-side copy is made from THIS repo (the one these scripts belong to). For another checkout given with
  # --repo the copy has nothing to do with it, so say nothing about refreshing it.
  if [ "$repo" = "$ASTRA_REPO_HERE" ]; then
    case "$repo" in
      /mnt/*) [ -d "$ASTRA_LINUX_COPY/ros2_ws/src" ] \
                && info "the containers use the Linux-side copy: refresh it now:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only" ;;
    esac
  fi
}
if [ "$FIX_EOL" = 1 ]; then fix_line_endings "$GIT_REPO"; exit 0; fi

# The download sites the image build (and the first run) needs: "address|what it is for". The apt sources are
# reached over plain http, exactly like the build does (their https certificates name other hosts).
NET_HOSTS=(
  "https://registry-1.docker.io/|Docker Hub: the ROS base image"
  "http://packages.ros.org/|ROS 2 packages"
  "http://snapshots.ros.org/|MAVROS (pinned ROS snapshot)"
  "https://keyserver.ubuntu.com/|the snapshot's signing key"
  "http://archive.ubuntu.com/|Ubuntu packages"
  "http://packages.osrfoundation.org/|Gazebo Harmonic"
  "https://github.com/|ArduPilot, mavlink-router, ardupilot_gazebo, detector weights"
  "https://objects.githubusercontent.com/|GitHub release files (onnxruntime, YOLO weights)"
  "https://raw.githubusercontent.com/|rosdep"
  "https://pypi.org/|Python packages"
  "https://files.pythonhosted.org/|Python package files"
  "https://download.pytorch.org/|PyTorch"
  "https://downloads.sourceforge.net/|MAVROS GeographicLib data"
  "https://fuel.gazebosim.org/|Gazebo 3D models (first run)"
)
# net_check : try every site at once (8 s each, in parallel); fills NET_BAD (unreachable) and NET_TLS (answered
# with a certificate chain this Linux does not trust = the network inspects HTTPS). Uses this terminal's proxy.
NET_BAD=(); NET_TLS=()
net_check() {
  local tmp entry url host label rc err i=0
  tmp="$(mktemp -d)"
  for entry in "${NET_HOSTS[@]}"; do
    url="${entry%%|*}"; i=$((i + 1))
    ( rc=0; curl -sS -o /dev/null -I --max-time 8 "$url" 2>"$tmp/$i.err" || rc=$?; echo "$rc" > "$tmp/$i.rc" ) &
  done
  wait
  i=0
  for entry in "${NET_HOSTS[@]}"; do
    url="${entry%%|*}"; label="${entry#*|}"; i=$((i + 1))
    host="${url#*://}"; host="${host%%/*}"
    rc="$(cat "$tmp/$i.rc" 2>/dev/null || echo 99)"
    err="$(head -1 "$tmp/$i.err" 2>/dev/null | sed 's/^curl: ([0-9]*) //')"
    if [ "$rc" = 0 ]; then continue; fi
    case "$err" in
      *"self-signed certificate"*|*"self signed certificate"*|*"local issuer certificate"*|*"certificate verify failed"*)
        NET_TLS+=("$host ($label)") ;;
      *) NET_BAD+=("$host ($label): $(printf '%s' "$err" | cut -c1-70)") ;;
    esac
  done
  rm -rf "$tmp"
}

echo "ASTRA Docker check for this computer (changes nothing; every problem says what to type)"
echo "------------------------------------------------------------------------------------------"

# =============================================================================================================
title "1. This computer"
case "$(uname -s 2>/dev/null)" in
  Linux) ;;
  Darwin)
    fail "this is macOS: the simulator image (astra-sim) needs Linux on an Intel/AMD processor, or Windows + WSL2"
    tip "the team dev container does work on a Mac: VS Code -> Reopen in Container -> \"ASTRA ODLC dev\""
    exit 1 ;;
  *)
    fail "this is not Linux (Git Bash, PowerShell or Cygwin on Windows?): the Docker scripts run inside Ubuntu"
    tip "Windows: open PowerShell, type   wsl -d Ubuntu-22.04   then go to the repo folder and run this again"
    tip "the Windows side is checked by:  powershell -ExecutionPolicy Bypass -File simulation\\docker\\check_host.ps1"
    exit 1 ;;
esac
IS_WSL=0
if astra_is_wsl; then
  IS_WSL=1
  if grep -qiE 'wsl2|microsoft-standard' /proc/version 2>/dev/null; then
    pass "Windows with WSL2, Linux distro \"${WSL_DISTRO_NAME:-?}\""
  else
    fail "this Linux runs on WSL version 1, where Docker cannot run"
    tip "in PowerShell:  wsl --set-version ${WSL_DISTRO_NAME:-Ubuntu-22.04} 2     (a few minutes), then open Ubuntu again"
  fi
else
  pass "a Linux PC (kernel $(uname -r))"
fi
if [ "$(id -u)" -eq 0 ]; then
  caution "you run this as root (with sudo?): run the Docker scripts as YOUR user, without sudo"
  tip "with sudo the home folder is /root, so the data folder and the repo copy are looked for in the wrong place"
  tip "Docker's \"permission denied\" is fixed by the docker group (section 3), not by sudo"
fi
[ -f /.dockerenv ] && caution "this runs INSIDE a container: run it on the computer itself (the WSL Ubuntu or the Linux PC)"
case "$(uname -m)" in
  x86_64|amd64) pass "Intel/AMD processor ($(uname -m)): all three images can be built here" ;;
  *) caution "processor type $(uname -m): only the dev image works here; the simulator image needs an Intel/AMD (x86_64) PC"
     tip "Gazebo's ROS packages are not made for $(uname -m): run the simulator on a Windows or Linux PC with an Intel/AMD processor" ;;
esac
OS_TEXT="$(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME:-unknown Linux}")"
OS_ID="$(. /etc/os-release 2>/dev/null && echo "${ID:-}-${VERSION_ID:-}")"
if [ "$OS_ID" = ubuntu-22.04 ]; then pass "$OS_TEXT"
else note "$OS_TEXT: Docker works on any Linux; install_docker_engine.sh and the native simulator need Ubuntu 22.04"; fi
MEM_T="$(awk '/^MemTotal:/ {print int($2 / 1024)}' /proc/meminfo 2>/dev/null)"
MEM_A="$(awk '/^MemAvailable:/ {print int($2 / 1024)}' /proc/meminfo 2>/dev/null)"
if [ "${MEM_T:-0}" -lt 7500 ]; then
  caution "only ${MEM_T:-?} MB of memory in this Linux: the simulator (about 3 GB) plus the detector (about 3 GB) need about 8 GB"
  if [ "$IS_WSL" = 1 ]; then
    tip "WSL gets half of the PC's memory by default. Give it more: on Windows create the file %UserProfile%\\.wslconfig with"
    tip "the two lines  [wsl2]  and  memory=12GB  then in PowerShell:  wsl --shutdown   and open Ubuntu again"
  fi
  tip "on a small PC use the light answer-key detector:  bash simulation/docker/compose_up.sh --detector truth"
else
  pass "memory: ${MEM_T} MB in total, ${MEM_A:-?} MB free right now"
  if [ "${MEM_A:-0}" -lt 4000 ]; then
    caution "only ${MEM_A:-?} MB of memory is free right now (a simulator run wants about 4000 MB)"
    tip "close big programs, or stop a simulator that still runs:  bash simulation/scripts/stop_sim.sh"
  fi
fi
note "$(nproc 2>/dev/null || echo ?) processor threads (build_images.sh --jobs 4 keeps the PC usable while it builds)"

# =============================================================================================================
title "2. Your copy of the code (the repository)"
note "checkout: $GIT_REPO"
[ -d "$GIT_REPO/ros2_ws/src" ] || fail "no ros2_ws/src in $GIT_REPO: this is not the ASTRA repository"
case "$GIT_REPO" in *" "*) note "the folder name has a space: fine for Docker (the scripts quote every path)" ;; esac
if [ "$MOUNT_REPO" != "$GIT_REPO" ]; then
  note "it is on a Windows drive, so the containers mount the Linux-side copy: $MOUNT_REPO"
  BEHIND="$(astra_copy_differences)"
  if [ "${BEHIND:-0}" -gt 0 ] 2>/dev/null; then
    caution "the Linux-side copy is OLDER than the checkout: $BEHIND file(s) changed since the last sync"
    tip "the containers would run the old code. Refresh the copy:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only   (or start with --sync)"
  else
    pass "the Linux-side copy matches the checkout"
  fi
else
  case "$GIT_REPO" in /mnt/[a-z]/*)
    note "it is on a Windows drive and there is no Linux-side copy yet: the containers mount the Windows folder (works, slower)"
    tip "make the copy once (faster builds, no path trouble):  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only" ;;
  esac
fi
# Line endings. A script git keeps with LF but which has CRLF on disk was turned into CRLF by a Windows clone, and
# cannot run in Linux at all. Fast path (reading every file through WSL's /mnt bridge takes about a minute): git
# lists the scripts from its own object store (shell scripts + files with a "#!" line), drops the few that were
# COMMITTED with CRLF (the repo's own state, they run through other means), and only those files are read from disk.
EOL_BY_GIT=0
GIT_ERR=""
if command -v git >/dev/null 2>&1 && GIT_ERR="$(git -C "$GIT_REPO" rev-parse --git-dir 2>&1 >/dev/null)"; then
  EOL_BY_GIT=1
  SCRIPTS=(); COMMITTED_CRLF=" "; CONV_SCRIPTS=()
  # (documents can show a "#!" line inside an example: they are not scripts)
  while IFS= read -r f; do [ -n "$f" ] && SCRIPTS+=("$f"); done < <(
    { git -C "$GIT_REPO" ls-files -- '*.sh' '*.bash'
      git -C "$GIT_REPO" grep --cached -I -l -e '^#!' -- . ':(exclude)*.md' ':(exclude)*.txt' ':(exclude)*.rst' \
        ':(exclude)*.html' ':(exclude)*.mhtml' ':(exclude)*.json' ':(exclude)*.yaml' ':(exclude)*.yml' 2>/dev/null; } | sort -u)
  if [ "${#SCRIPTS[@]}" -gt 0 ]; then
    while IFS= read -r f; do COMMITTED_CRLF="$COMMITTED_CRLF$f "; done < <(
      git -C "$GIT_REPO" grep --cached -I -l -e $'\r$' -- "${SCRIPTS[@]}" 2>/dev/null)
    while IFS= read -r f; do
      f="${f#"$GIT_REPO"/}"
      case "$COMMITTED_CRLF" in *" $f "*) ;; *) CONV_SCRIPTS+=("$f") ;; esac
    done < <(cd "$GIT_REPO" && grep -l -I $'\r$' -- "${SCRIPTS[@]}" 2>/dev/null)
  fi
  if [ "${#CONV_SCRIPTS[@]}" -gt 0 ]; then
    # Name the most important one: the mission brain if it is affected, else the first that is not a backup copy.
    EXAMPLE=""
    for f in "${CONV_SCRIPTS[@]}"; do case "$f" in */main_controller.py) EXAMPLE="$f"; break ;; esac; done
    if [ -z "$EXAMPLE" ]; then for f in "${CONV_SCRIPTS[@]}"; do case "$f" in *.bak*) ;; *) EXAMPLE="$f"; break ;; esac; done; fi
    EXAMPLE="${EXAMPLE:-${CONV_SCRIPTS[0]}}"
    fail "${#CONV_SCRIPTS[@]} script(s) have Windows line endings (CRLF), e.g. $EXAMPLE"
    tip "in Linux they stop with  \$'\\r': command not found  or  /usr/bin/env: 'python3\\r': No such file or directory"
    tip "cause: the repo was cloned on Windows, where Git's default (core.autocrlf=true) writes CRLF. Repair it (keeps your edits):"
    tip "    bash $ASTRA_DOCKER_SCRIPTS/check_host.sh --fix-line-endings$([ -n "$REPO_ARG" ] && printf ' --repo %q' "$GIT_REPO")"
    tip "the repo's .gitattributes makes git write LF from now on (git pull first if your copy is older)"
  else
    pass "line endings: the ${#SCRIPTS[@]} scripts git keeps with Linux line endings (LF) have them on disk"
  fi
elif printf '%s' "$GIT_ERR" | grep -q 'dubious ownership'; then
  caution "git refuses to read $GIT_REPO (\"dubious ownership\": the folder belongs to another user)"
  tip "allow it:  git config --global --add safe.directory \"$GIT_REPO\""
else
  note "$GIT_REPO is not a git checkout (a ZIP download?): only the shell scripts are checked"
fi
# Whatever git says: the shell scripts in the folder the containers mount must have LF.
CRLF_SH=()
while IFS= read -r f; do [ -n "$f" ] && CRLF_SH+=("$f"); done < <(grep -rlI $'\r$' --include='*.sh' --include='*.bash' \
  --exclude-dir=.git --exclude-dir=build --exclude-dir=install --exclude-dir=log --exclude-dir=__pycache__ \
  --exclude-dir=mapping_photos --exclude-dir=detection_results_sahi --exclude-dir=terrain "$MOUNT_REPO" 2>/dev/null)
if [ "${#CRLF_SH[@]}" -gt 0 ]; then
  fail "${#CRLF_SH[@]} shell script(s) in $MOUNT_REPO have Windows line endings, e.g. ${CRLF_SH[0]#"$MOUNT_REPO"/}"
  if [ "$MOUNT_REPO" != "$GIT_REPO" ]; then
    tip "repair the checkout (see above), then refresh the copy:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only"
  elif [ "$EOL_BY_GIT" = 0 ]; then
    tip "repair one file:  sed -i 's/\\r\$//' FILE      (or get the repo again with  git clone  inside Ubuntu)"
  fi
elif [ "$EOL_BY_GIT" = 0 ] || [ "$MOUNT_REPO" != "$GIT_REPO" ]; then
  pass "the shell scripts in $MOUNT_REPO have Linux line endings (LF)"
fi
# Folders Docker made for a container (owned by root) block your own builds and runs.
for d in "$MOUNT_REPO/ros2_ws/build" "$MOUNT_REPO/ros2_ws/install" "$MOUNT_REPO/ros2_ws/log" "$ASTRA_DOCKER_DATA"; do
  [ -e "$d" ] || continue
  o="$(stat -c %u "$d" 2>/dev/null)"
  if [ -n "$o" ] && [ "$o" != "$(id -u)" ]; then
    caution "$d belongs to user id $o, not to you ($(id -u)): Docker made it for a container; your builds and runs cannot write there"
    tip "give it back:  sudo chown -R $(id -u):$(id -g) \"$d\""
  fi
done

# =============================================================================================================
title "3. Docker"
ENGINE_OK=0; DESKTOP=0; DOCKER_CLI=""
WHERE="$(command -v docker 2>/dev/null || true)"
DD_LINK=""                      # Docker Desktop's WSL integration links its own docker program into the distro
for f in /usr/bin/docker /usr/local/bin/docker; do
  if [ -L "$f" ] && readlink "$f" 2>/dev/null | grep -q docker-desktop; then DD_LINK="$f"; fi
done
if [ -n "$DD_LINK" ] && { [ -z "$WHERE" ] || [ "${WHERE#/mnt/}" != "$WHERE" ]; }; then
  fail "Docker Desktop's WSL integration is on, but Docker Desktop is not running ($DD_LINK points nowhere)"
  tip "start Docker Desktop on Windows (Start menu -> Docker Desktop) and wait until it says \"Engine running\"; then run this again"
elif [ -z "$WHERE" ]; then
  fail "Docker is not installed in this Linux"
  if [ "$IS_WSL" = 1 ]; then
    tip "EITHER install Docker Engine inside this Ubuntu (our tested way; images stay on this distro's disk):"
    tip "    sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh"
    tip "OR use Docker Desktop: Docker Desktop -> Settings -> Resources -> WSL integration -> switch on \"${WSL_DISTRO_NAME:-Ubuntu-22.04}\" -> Apply"
  else
    tip "Ubuntu:  sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh      other Linux: https://docs.docker.com/engine/install/"
  fi
else
  case "$WHERE" in
    /mnt/*)
      fail "\"docker\" here is Docker Desktop's Windows helper ($WHERE): it only prints \"activate the WSL integration\""
      tip "EITHER switch on Docker Desktop's WSL integration: Settings -> Resources -> WSL integration -> \"${WSL_DISTRO_NAME:-Ubuntu-22.04}\" -> Apply & restart"
      tip "OR install Docker Engine inside this Ubuntu:  sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh   (then open a NEW terminal)" ;;
    *) DOCKER_CLI="$WHERE" ;;
  esac
fi
[ -n "${DOCKER_HOST:-}" ] && note "DOCKER_HOST=$DOCKER_HOST is set: docker talks to that engine, not this computer's usual one"
if [ -n "$DOCKER_CLI" ]; then
  INFO_ERR="$(timeout 40 docker info 2>&1 >/dev/null)"; rc=$?
  LOOKS_DESKTOP=0
  { [ -n "$DD_LINK" ] || readlink -f "$DOCKER_CLI" 2>/dev/null | grep -q docker-desktop; } && LOOKS_DESKTOP=1
  if [ "$rc" = 0 ]; then
    ENGINE_OK=1
  elif [ "$rc" = 124 ]; then
    fail "Docker did not answer within 40 s"
    tip "Docker Desktop: wait until its window says \"Engine running\".   Docker Engine:  sudo systemctl restart docker"
  else
    case "$INFO_ERR" in
      *"ermission denied"*)
        fail "your user may not use Docker yet (\"permission denied\" on /var/run/docker.sock)"
        if id -nG "$(id -un)" 2>/dev/null | tr ' ' '\n' | grep -qx docker; then
          tip "you ARE in the docker group, but this terminal was opened before that: open a new login (below)"
        else
          tip "add yourself once:  sudo usermod -aG docker $(id -un)"
        fi
        if [ "$IS_WSL" = 1 ]; then
          tip "then close every Ubuntu window, run  wsl --terminate ${WSL_DISTRO_NAME:-Ubuntu-22.04}  in PowerShell, and open Ubuntu again"
        else
          tip "then log out and log in again (or restart the PC)"
        fi
        tip "right now, for one command:  sg docker -c \"bash $ASTRA_DOCKER_SCRIPTS/check_host.sh\"" ;;
      *)
        if [ "$LOOKS_DESKTOP" = 1 ]; then
          fail "Docker Desktop is not running (or still starting)"
          tip "start Docker Desktop on Windows and wait until it says \"Engine running\""
        else
          fail "Docker Engine is installed but not running: $(printf '%s' "$INFO_ERR" | grep -v '^$' | tail -1 | cut -c1-90)"
          if [ "$(ps -p 1 -o comm= 2>/dev/null)" != systemd ]; then
            tip "systemd is off in this distro, and Docker Engine needs it: add  [boot]  and  systemd=true  to /etc/wsl.conf"
            tip "(sudo nano /etc/wsl.conf), then in PowerShell:  wsl --terminate ${WSL_DISTRO_NAME:-Ubuntu-22.04}   and open Ubuntu again"
          else
            tip "start it:  sudo systemctl start docker        (what went wrong:  journalctl -u docker --no-pager | tail -40)"
          fi
        fi ;;
    esac
  fi
fi
if [ "$ENGINE_OK" = 1 ]; then
  IFS='|' read -r SV OSN ROOTD HPX HSPX <<< "$(timeout 20 docker info --format '{{.ServerVersion}}|{{.OperatingSystem}}|{{.DockerRootDir}}|{{.HTTPProxy}}|{{.HTTPSProxy}}' 2>/dev/null)"
  case "$OSN" in *"Docker Desktop"*) DESKTOP=1 ;; esac
  pass "Docker answers: engine $SV, $([ "$DESKTOP" = 1 ] && echo "Docker Desktop" || echo "Docker Engine on ${OSN:-?}")"
  if [ "${SV%%.*}" -lt 23 ] 2>/dev/null; then
    caution "Docker $SV is old (23 or newer has BuildKit and a current Compose)"
    tip "$([ "$DESKTOP" = 1 ] && echo "update Docker Desktop" || echo "sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh")"
  fi
  BX="$(timeout 20 docker buildx version 2>/dev/null | head -1)"
  if [ -n "$BX" ]; then
    pass "build tool: buildx (BuildKit) $(printf '%s' "$BX" | awk '{print $2}')"
  else
    fail "Docker's build tool \"buildx\" (BuildKit) is missing: Docker would use its old builder, which builds the sitl/sim images wrongly"
    tip "Docker's packages:  sudo apt-get install docker-buildx-plugin      Ubuntu's docker.io:  sudo apt-get install docker-buildx"
  fi
  CV="$(timeout 20 docker compose version --short 2>/dev/null)"; CV="${CV#v}"
  if [ -z "$CV" ]; then
    fail "Docker Compose (the \"docker compose\" command) is missing"
    tip "sudo apt-get install docker-compose-plugin      (Docker's apt source; install_docker_engine.sh sets it up)"
    command -v docker-compose >/dev/null 2>&1 && tip "the old separate docker-compose (version 1) is not enough: the scripts need \"docker compose\" 2.20 or newer"
  else
    CMAJ="${CV%%.*}"; CMIN="$(printf '%s' "$CV" | cut -d. -f2)"
    if [ "$CMAJ" -lt 2 ] 2>/dev/null || { [ "$CMAJ" -eq 2 ] 2>/dev/null && [ "${CMIN:-0}" -lt 20 ] 2>/dev/null; }; then
      fail "Docker Compose $CV is too old (2.20 or newer is needed)"
      tip "$([ "$DESKTOP" = 1 ] && echo "update Docker Desktop" || echo "sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh")"
    else
      pass "Docker Compose $CV"
    fi
  fi
  if [ "$DESKTOP" = 1 ] && dpkg -s docker-ce >/dev/null 2>&1; then
    caution "Docker Desktop's WSL integration AND Docker Engine (docker-ce) are both in this distro: two Dockers fight over \"docker\""
    tip "keep one: Docker Desktop -> Settings -> Resources -> WSL integration -> switch this distro off (to keep the Engine)"
  fi
  if [ "$DESKTOP" = 0 ] && [ "$IS_WSL" = 1 ] && [ -e "/mnt/c/Program Files/Docker/Docker/Docker Desktop.exe" ]; then
    note "Docker Desktop is also installed on Windows: keep its WSL integration for this distro switched off (this distro uses its own Engine)"
  fi
  CTX="$(timeout 10 docker context show 2>/dev/null)"
  case "$CTX" in ''|default|desktop-linux) ;; *) note "docker context \"$CTX\" is active (back to the normal one:  docker context use default)" ;; esac
  # A credential helper that does not exist breaks every download, even of public images.
  CFG="${DOCKER_CONFIG:-$HOME/.docker}/config.json"
  if [ -f "$CFG" ]; then
    HELPERS="$(python3 - "$CFG" 2>/dev/null <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
names = [d.get('credsStore') or ''] + list((d.get('credHelpers') or {}).values())
print(' '.join(sorted({n for n in names if n})))
PY
)"
    for h in $HELPERS; do
      if ! command -v "docker-credential-$h" >/dev/null 2>&1; then
        fail "$CFG names the password helper \"$h\", which is not installed: every download fails (\"error getting credentials\")"
        tip "remove that setting (no login is needed for our images):"
        tip "python3 -c \"import json,os;p=os.path.expanduser('$CFG');d=json.load(open(p));d.pop('credsStore',None);d.pop('credHelpers',None);json.dump(d,open(p,'w'),indent=2)\""
      fi
    done
  fi
  # Disk space where the images go.
  MISSING_ANY=0
  for i in astra-dev:latest astra-sitl:latest astra-sim:latest; do astra_image_exists "$i" || MISSING_ANY=1; done
  if [ "$DESKTOP" = 1 ]; then
    note "Docker Desktop keeps images in its own virtual disk: Docker Desktop -> Settings -> Resources -> Advanced (disk image location and size)"
  else
    FREE_GB="$(astra_free_gb "${ROOTD:-/var/lib/docker}")"
    NEED=10; [ "$MISSING_ANY" = 1 ] && NEED=40
    if [ -z "$FREE_GB" ]; then
      note "could not read the free space of ${ROOTD:-/var/lib/docker}"
    elif [ "$FREE_GB" -lt 25 ] && [ "$MISSING_ANY" = 1 ]; then
      fail "only $FREE_GB GB free where Docker keeps images (${ROOTD:-/var/lib/docker}): building needs about 40 GB"
      tip "free space (Windows: the drive that holds the WSL disk), and remove old build data:  docker builder prune"
    elif [ "$FREE_GB" -lt "$NEED" ]; then
      caution "only $FREE_GB GB free where Docker keeps images (${ROOTD:-/var/lib/docker}); about $NEED GB is recommended"
      tip "see what Docker uses:  docker system df      remove old build data:  docker builder prune"
    else
      pass "$FREE_GB GB free where Docker keeps images (${ROOTD:-/var/lib/docker})"
    fi
  fi
  # The compose files, checked by Docker Compose itself with the add-on files this computer would use.
  astra_compose_files "$(astra_display_mode)" auto
  if C_OUT="$(cd "$GIT_REPO" && ASTRA_REPO_HOST="$MOUNT_REPO" timeout 60 docker compose -p astra "${COMPOSE_FILES[@]}" config --quiet 2>&1)"; then
    pass "the compose files are valid for this computer ($COMPOSE_TEXT)"
  else
    fail "docker compose rejects the compose files: $(printf '%s' "$C_OUT" | grep -v '^$' | tail -1 | cut -c1-110)"
    tip "a newer Compose may be needed (2.20+), or a file was damaged: git -C \"$GIT_REPO\" diff -- docker/"
  fi
  # Leftovers that make "compose up" stop.
  LEFT="$(timeout 10 docker ps --format '{{.Names}}' --filter 'name=^astra-' 2>/dev/null | tr '\n' ' ')"
  if [ -n "${LEFT// /}" ]; then
    caution "ASTRA containers are still running: $LEFT"
    tip "stop them before a new start:  bash $ASTRA_DOCKER_SCRIPTS/compose_down.sh --solo"
  fi
  while read -r cname cproj; do
    [ -n "$cname" ] || continue
    if [ "$cproj" != astra ]; then
      fail "a container called $cname exists that docker compose did not make: compose up stops with \"The container name ... is already in use\""
      tip "remove it:  docker rm -f $cname"
    fi
  done < <(timeout 10 docker ps -a --filter 'name=^astra-(sim|jetson)$' --format '{{.Names}} {{.Label "com.docker.compose.project"}}' 2>/dev/null)
  if timeout 10 docker network inspect "$ASTRA_NET" >/dev/null 2>&1; then
    NSUB="$(docker network inspect "$ASTRA_NET" --format '{{range .IPAM.Config}}{{.Subnet}} {{end}}' 2>/dev/null)"; NSUB="${NSUB% }"
    NLBL="$(docker network inspect "$ASTRA_NET" --format '{{index .Labels "com.docker.compose.network"}}' 2>/dev/null)"
    NCNT="$(docker network inspect "$ASTRA_NET" --format '{{len .Containers}}' 2>/dev/null)"
    if [ "$NSUB" != "$ASTRA_SUBNET" ] || [ "$NLBL" != "$ASTRA_NET" ]; then
      fail "a Docker network \"$ASTRA_NET\" exists that compose did not make (subnet ${NSUB:-?}): compose up stops (\"incorrect label\")"
      [ "${NCNT:-0}" != 0 ] && tip "first stop the ${NCNT} container(s) on it:  bash $ASTRA_DOCKER_SCRIPTS/compose_down.sh --solo"
      tip "then remove it:  docker network rm $ASTRA_NET      (compose_up.sh makes the right one)"
    else
      note "the drone network $ASTRA_NET exists (${NCNT:-0} container(s) on it); compose_down.sh removes it after a run"
    fi
  fi
fi
CONFLICT="$(astra_subnet_conflict)"
if [ -n "$CONFLICT" ]; then
  fail "the drone network $ASTRA_SUBNET is already used on this computer by: $(printf '%s' "$CONFLICT" | head -1)"
  tip "the containers need exactly these addresses (the team's code expects the camera at $ASTRA_CAMERA_IP). Free them:"
  tip "unplug / switch off the network card that is on the drone's network (or a VPN using it), or  docker network rm <that network>"
else
  pass "the drone network $ASTRA_SUBNET is free for the containers"
fi
WIDER="$(astra_subnet_wider_route)"
if [ -n "$WIDER" ]; then
  caution "a wider network route covers the drone network: $WIDER"
  tip "the containers still start, but while they run this computer cannot reach 192.168.144.x on that network"
  tip "(a VPN, or a big home/school network). Fine for the simulator; if that IS the real drone's network, unplug it while you use the containers"
fi

# =============================================================================================================
title "4. The ASTRA images (built on this PC from docker/Dockerfile; never downloaded, no login needed)"
MISSING=()
if [ "$ENGINE_OK" = 1 ]; then
  WANT_HASH="$(astra_docker_inputs_hash)"
  for t in dev sitl sim; do
    img="astra-$t:latest"
    if ! astra_image_exists "$img"; then MISSING+=("$t"); continue; fi
    created="$(docker image inspect "$img" --format '{{.Created}}' 2>/dev/null | cut -c1-16 | tr T ' ')"
    iuser="$(docker image inspect "$img" --format '{{.Config.User}}' 2>/dev/null)"
    iuid="$(astra_image_label "$img" org.astra.uid)"
    iin="$(astra_image_label "$img" org.astra.inputs)"
    itorch="$(astra_image_label "$img" org.astra.torch)"
    line="$img (built $created$([ -n "$itorch" ] && echo ", PyTorch $itorch"))"
    if [ -z "$iuser" ] || [ "$iuser" = root ] || [ "$iuser" = 0 ]; then
      fail "$line runs as root (built by Docker's old builder?): runs, logs and hand-over files would vanish in /root"
      tip "rebuild it with the helper (it insists on buildx):  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh $t"
      continue
    fi
    if [ -n "$iin" ] && [ "$iin" != "$WANT_HASH" ]; then
      caution "$line is OLDER than docker/Dockerfile or docker/requirements-*.txt (they changed after it was built)"
      tip "rebuild it (unchanged steps are reused):  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh $t"
    elif [ -z "$iin" ]; then
      pass "$line"
      note "    built without build_images.sh's labels: this check cannot tell whether it is up to date"
    else
      pass "$line, up to date with docker/"
    fi
    if [ -n "$iuid" ] && [ "$iuid" != "$(id -u)" ]; then
      if [ "$iuid" != "$(astra_build_uid)" ]; then
        caution "$img was built for user id $iuid, you are $(id -u): files it writes into the repo will belong to $iuid"
        tip "rebuild it with your id:  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh $t"
      else
        # build_images.sh never uses ids below 1000 (they clash with system groups in the image): a rebuild
        # would give the same image, so there is nothing to do.
        note "$img uses user id $iuid; yours ($(id -u)) is below 1000, so the images always use 1000 (nothing to rebuild)"
      fi
    fi
  done
  if [ "${#MISSING[@]}" -eq 3 ]; then
    caution "no ASTRA images on this PC yet (normal on a new PC). Build them once (about 1 hour, about 25 GB):"
    tip "bash $ASTRA_DOCKER_SCRIPTS/build_images.sh        (or load a teammate's: DOCKER.md, \"Copy images without rebuilding\")"
    tip "Docker's \"pull access denied ... may require 'docker login'\" for astra-* only means \"not built yet\": no login is needed"
  elif [ "${#MISSING[@]}" -gt 0 ]; then
    caution "not built yet: $(printf 'astra-%s ' "${MISSING[@]}")"
    tip "bash $ASTRA_DOCKER_SCRIPTS/build_images.sh ${MISSING[*]}"
  fi
  BASE_DATE="$(docker image inspect ros:humble-ros-base-jammy --format '{{.Created}}' 2>/dev/null | cut -c1-10)"
  if [ -n "$BASE_DATE" ] && [[ "$BASE_DATE" < 2025-06-01 ]]; then
    caution "the saved ROS base image is from $BASE_DATE: it still trusts the ROS signing key that expired in June 2025 (EXPKEYSIG)"
    tip "build_images.sh fetches a fresh one by itself; by hand:  docker pull ros:humble-ros-base-jammy"
  fi
else
  note "skipped: Docker does not answer (section 3)"
fi

# =============================================================================================================
title "5. Internet (the first build downloads about 10 GB), proxy and clock"
NEEDS_BUILD=0; { [ "$ENGINE_OK" = 0 ] || [ "${#MISSING[@]}" -gt 0 ]; } && NEEDS_BUILD=1
PX=""
for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do [ -n "${!v:-}" ] && PX="$PX $v"; done
[ -n "$PX" ] && note "proxy settings in this terminal:$PX (build_images.sh passes them into the build)"
if [ -n "$PX" ] && [ "$ENGINE_OK" = 1 ] && [ "$DESKTOP" = 0 ] && [ -z "${HPX:-}${HSPX:-}" ]; then
  caution "this terminal uses a proxy, but Docker Engine has none: it cannot download the ROS base image through it"
  tip "give the Engine the same proxy: sudo systemctl edit docker   add the two lines  [Service]  and"
  tip "Environment=\"HTTPS_PROXY=${https_proxy:-${HTTPS_PROXY:-http://proxy:port}}\"   save, then:  sudo systemctl restart docker"
fi
if [ "$OFFLINE" = 1 ]; then
  note "internet checks skipped (--offline)"
elif ! command -v curl >/dev/null 2>&1; then
  note "curl is missing, internet not checked:  sudo apt-get install curl"
elif ! getent hosts github.com >/dev/null 2>&1; then
  fail "this Linux cannot look up internet names (github.com): no internet, or name lookup (DNS) is broken"
  [ "$IS_WSL" = 1 ] && tip "WSL: a VPN often breaks this. Disconnect it, or add  [wsl2]  dnsTunneling=true  to %UserProfile%\\.wslconfig, then  wsl --shutdown"
  tip "check:  cat /etc/resolv.conf      try:  ping -c1 8.8.8.8   (works = only DNS is broken)"
else
  net_check
  if [ "${#NET_BAD[@]}" -eq 0 ] && [ "${#NET_TLS[@]}" -eq 0 ]; then
    pass "all ${#NET_HOSTS[@]} download sites answer (Docker Hub, ROS, Ubuntu, Gazebo, GitHub, PyPI, PyTorch, SourceForge, Fuel)"
  fi
  for b in "${NET_BAD[@]}"; do
    if [ "$NEEDS_BUILD" = 1 ]; then fail "cannot reach $b"; else caution "cannot reach $b"; fi
  done
  [ "${#NET_BAD[@]}" -gt 0 ] && tip "a school or company network may block these: build on another network (home, phone hotspot), or try again later"
  if [ "${#NET_TLS[@]}" -gt 0 ]; then
    caution "these sites answered with a certificate this Linux does not trust: ${NET_TLS[*]}"
    tip "your network inspects HTTPS (school/company proxy). Build on another network, or ask IT for their root certificate"
  fi
  # Clock: apt refuses package lists from "the future" ("Release file ... is not valid yet"); WSL's clock can fall
  # behind after the laptop slept.
  DH="$(curl -sSI --max-time 8 https://github.com/ 2>/dev/null | tr -d '\r' | awk -F': ' 'tolower($1) == "date" {print $2; exit}')"
  REMOTE=""
  [ -n "$DH" ] && REMOTE="$(date -d "$DH" +%s 2>/dev/null)"     # (date -d "" would mean "today 00:00")
  if [ -n "$REMOTE" ]; then
    SKEW=$(( $(date +%s) - REMOTE )); ASKEW=${SKEW#-}
    if [ "$ASKEW" -gt 3600 ]; then
      fail "this Linux's clock is off by $ASKEW s: package downloads fail (\"Release file ... is not valid yet\")"
    elif [ "$ASKEW" -gt 120 ]; then
      caution "this Linux's clock is off by $ASKEW s (apt may refuse package lists)"
    else
      pass "the clock is right (within $ASKEW s)"
    fi
    if [ "$ASKEW" -gt 120 ]; then
      if [ "$IS_WSL" = 1 ]; then tip "set it from Windows' clock:  sudo hwclock -s      (or in PowerShell:  wsl --shutdown, then open Ubuntu again)"
      else tip "turn on time sync:  sudo timedatectl set-ntp true"; fi
    fi
  fi
fi

# =============================================================================================================
title "6. Windows on the screen, 3D graphics and the graphics card"
if [ "$IS_WSL" = 1 ]; then
  if [ -d /mnt/wslg ]; then
    if [ -S /tmp/.X11-unix/X0 ]; then
      pass "WSLg is there: Gazebo, RViz and the panel can open windows on the Windows desktop"
    elif [ -S /mnt/wslg/.X11-unix/X0 ]; then
      caution "WSLg runs, but its window socket is missing from /tmp/.X11-unix (a known WSL + systemd problem): no windows"
      tip "fix for now:  sudo ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0"
      tip "fix for good:  echo 'L+ /tmp/.X11-unix/X0 - - - - /mnt/wslg/.X11-unix/X0' | sudo tee /etc/tmpfiles.d/wslg.conf"
    else
      caution "WSLg is installed but its window server did not start (no X0 socket)"
      tip "in PowerShell:  wsl --update   then  wsl --shutdown   and open Ubuntu again"
    fi
  else
    caution "no WSLg (/mnt/wslg is missing): the containers can only run without windows (--headless)"
    tip "Windows 11 (or Windows 10 22H2): in PowerShell  wsl --update   then  wsl --shutdown   and open Ubuntu again"
  fi
  [ -n "${DISPLAY:-}" ] || { caution "DISPLAY is not set in this terminal (WSLg normally sets it to :0)"; tip "export DISPLAY=:0"; }
  if [ -e /dev/dxg ]; then
    pass "/dev/dxg is there: Gazebo and RViz in the containers draw on the graphics card (Mesa d3d12)"
  else
    caution "no /dev/dxg: 3D is drawn by the processor (slow); --detector truth keeps a run light"
    tip "update the graphics driver on Windows, then in PowerShell:  wsl --update"
  fi
else
  if [ -n "${DISPLAY:-}" ]; then
    n="${DISPLAY#*:}"; n="${n%%.*}"
    if [ -S "/tmp/.X11-unix/X$n" ]; then
      pass "X11 display $DISPLAY: the containers can open windows on this desktop"
      command -v xhost >/dev/null 2>&1 || { caution "xhost is missing: the X server may refuse the containers' windows"; tip "sudo apt-get install x11-xserver-utils"; }
    else
      caution "DISPLAY=$DISPLAY, but there is no socket /tmp/.X11-unix/X$n (an SSH display?): containers cannot open windows there"
      tip "use --headless, or run from a terminal on the PC's own desktop"
    fi
  elif [ -n "${WAYLAND_DISPLAY:-}" ]; then
    caution "a Wayland desktop without X11 (no DISPLAY): Gazebo/RViz/Qt windows in the containers need XWayland"
    tip "sudo apt-get install xwayland   and log in again, or choose \"Ubuntu on Xorg\" on the login screen"
  else
    note "no desktop in this terminal (SSH?): the scripts run the containers without windows (--headless)"
  fi
  if [ -e /dev/dri/renderD128 ]; then
    pass "/dev/dri/renderD128 (group $(stat -c %G /dev/dri/renderD128 2>/dev/null)): 3D on this PC's graphics chip"
  else
    note "no /dev/dri render node: 3D is drawn by the processor unless an NVIDIA card is given to the containers"
  fi
fi
HOST_GPU=""
if [ -e /usr/lib/wsl/lib/libcuda.so.1 ] || nvidia-smi -L >/dev/null 2>&1; then
  HOST_GPU="$(nvidia-smi -L 2>/dev/null | head -1 | sed 's/ (UUID.*//')"; HOST_GPU="${HOST_GPU:-an NVIDIA card}"
fi
if [ -z "$HOST_GPU" ]; then
  note "no NVIDIA card seen: the detector runs on the processor (slower; --detector truth is the light choice)"
elif [ "$ENGINE_OK" = 1 ] && astra_has_nvidia_runtime; then
  pass "NVIDIA: $HOST_GPU, and Docker can give it to the containers (nvidia runtime)"
  for t in dev sim; do
    if [ "$(astra_image_label "astra-$t:latest" org.astra.torch)" = cpu ]; then
      note "astra-$t has CPU PyTorch: the detector will not use the card. Rebuild:  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh --torch cuda"
      break
    fi
  done
elif [ "$ENGINE_OK" = 1 ]; then
  caution "NVIDIA card found ($HOST_GPU), but Docker cannot give it to containers: the detector runs on the processor (about 10x slower)"
  if [ "$DESKTOP" = 1 ]; then
    tip "Docker Desktop: update Docker Desktop and the NVIDIA driver on Windows; check:  docker run --rm --gpus all ubuntu:22.04 nvidia-smi"
  else
    tip "install the NVIDIA Container Toolkit:  sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh --nvidia"
  fi
else
  note "NVIDIA card found ($HOST_GPU); whether Docker can use it is checked once Docker answers"
fi

# =============================================================================================================
title "7. Things that would get in the way of a run"
if astra_native_sim_running; then
  caution "a NATIVE simulator (Gazebo / SITL outside Docker) is running now: compose_up.sh will not start next to it"
  tip "stop it first:  bash $GIT_REPO/simulation/scripts/stop_sim.sh"
else
  pass "no native simulator is running"
fi
PORTS=""
if command -v ss >/dev/null 2>&1; then
  SS_OUT="$(ss -H -lunpt 2>/dev/null)"
  for p in udp:14550 udp:14552 tcp:5760 tcp:5762 tcp:5763 tcp:8081 tcp:82; do
    proto="${p%%:*}"; port="${p#*:}"
    hit="$(printf '%s\n' "$SS_OUT" | awk -v pr="$proto" -v pt="$port" '$1 == pr { n = split($5, a, ":"); if (a[n] == pt) { print; exit } }')"
    [ -n "$hit" ] && PORTS="$PORTS $proto/$port$(printf '%s' "$hit" | grep -o '(("[^"]*"' | head -1 | tr -d '("' | sed 's/^/=/')"
  done
fi
if [ -n "$PORTS" ]; then
  note "ports in use on this computer:$PORTS"
  tip "no problem for compose_up.sh (the containers have their own network), but they clash with"
  tip "run_jetson_container.sh --net host and with a second native simulator (stop it: simulation/scripts/stop_sim.sh)"
fi
astra_camera_ip_on_host && note "this computer has the camera address $ASTRA_CAMERA_IP (native simulator): compose_up.sh takes it away for the run (sudo), compose_down.sh --restore-camera-ip gives it back"
W1="$ASTRA_DOCKER_DATA/models/yolo26m.pt"; W2="$HOME/.astra_sim/models/yolo26m.pt"
if [ -s "$W1" ]; then pass "detector weights: $W1"
elif [ -s "$W2" ]; then pass "detector weights: $W2 (compose_up.sh copies them into $ASTRA_DOCKER_DATA)"
else note "no detector weights yet (yolo26m.pt, 40 MB): downloaded on the first real-detector run (github.com). Offline: --detector truth, or copy the file to $W1"
fi
DATA_FREE="$(astra_free_gb "$ASTRA_DOCKER_DATA")"
if [ -n "$DATA_FREE" ] && [ "$DATA_FREE" -lt 5 ]; then
  caution "only $DATA_FREE GB free for the run folders ($ASTRA_DOCKER_DATA): each run keeps about 100 photos"
  tip "move old runs away:  ls $ASTRA_DOCKER_DATA/runs"
fi

# =============================================================================================================
if [ "$DEEP" = 1 ]; then
  title "8. Deep checks (small test containers)"
  if [ "$ENGINE_OK" = 0 ]; then
    note "skipped: Docker does not answer"
  else
    PRESENT=()
    for t in dev sitl sim; do astra_image_exists "astra-$t:latest" && PRESENT+=("$t"); done
    if [ "${#PRESENT[@]}" -gt 0 ]; then
      if bash "$ASTRA_DOCKER_SCRIPTS/build_images.sh" --check "${PRESENT[@]}"; then pass "the images' self-test passed (${PRESENT[*]})"
      else fail "the images' self-test found a problem (FAIL lines above)"; tip "rebuild the image named there:  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh <dev|sitl|sim>"; fi
      if timeout 60 docker run --rm --entrypoint getent astra-"${PRESENT[0]}":latest hosts github.com >/dev/null 2>&1; then
        pass "containers can look up internet names (github.com)"
      else
        fail "containers cannot look up internet names: the Fuel models and detector weights downloads will fail"
        tip "Engine: add  \"dns\": [\"8.8.8.8\", \"1.1.1.1\"]  to /etc/docker/daemon.json, then  sudo systemctl restart docker"
      fi
    else
      note "no ASTRA image to test yet"
    fi
    if astra_has_nvidia_runtime; then
      if timeout 180 docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L >/dev/null 2>&1; then pass "a container can use the NVIDIA card (docker run --gpus all ... nvidia-smi)"
      else fail "a container could not use the NVIDIA card"; tip "see:  docker run --rm --gpus all ubuntu:22.04 nvidia-smi"; fi
    fi
  fi
fi

echo
echo "------------------------------------------------------------------------------------------"
if [ "$FAILS" -gt 0 ]; then
  echo "$FAILS problem(s) to fix first (the [FAIL] lines, each with the line to type), $WARNS warning(s). Then run this check again."
  echo "Every message is explained in docs/simulation/DOCKER.md, \"Troubleshooting\"."
  exit 1
fi
if [ "$ENGINE_OK" = 1 ] && [ "${#MISSING[@]}" -gt 0 ]; then
  echo "Nothing blocks Docker here ($WARNS warning(s)). Next step, the images:  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh"
else
  echo "Nothing blocks a Docker run here ($WARNS warning(s) above). Next:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh runway --dry-run"
fi
echo "Every message is explained in docs/simulation/DOCKER.md, \"Troubleshooting\"."
exit 0
