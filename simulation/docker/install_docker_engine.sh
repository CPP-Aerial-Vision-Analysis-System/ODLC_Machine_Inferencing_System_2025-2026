#!/usr/bin/env bash
# =============================================================================
#  install_docker_engine.sh  --  ONE-TIME: install Docker Engine (+ NVIDIA GPU support) inside Ubuntu
#
#  PURPOSE: gets this Ubuntu (the Ubuntu-22.04 WSL distro on Windows, or a normal Linux PC) ready to build
#  and run the ASTRA containers (docs/simulation/DOCKER.md). It installs the free Docker ENGINE from Docker's
#  own apt source, NOT Docker Desktop, so on Windows every image lives inside this distro's virtual disk
#  (for us D:\WSL\Ubuntu-22.04\ext4.vhdx) instead of Docker Desktop's disk on C:. Step by step it:
#    1. adds Docker's apt source and installs docker-ce, docker-ce-cli, containerd.io, docker-buildx-plugin,
#       docker-compose-plugin and pigz (unpacks images faster);
#    2. when an NVIDIA graphics card is visible, adds NVIDIA's apt source, installs the NVIDIA Container
#       Toolkit and registers it with Docker, so `docker run --gpus all ...` can use CUDA (PyTorch);
#    3. writes /etc/docker/daemon.json with size-limited log files (a chatty container could fill the disk);
#    4. inside WSL: Docker starts the first time you use it (systemd "socket activation"), so it costs no
#       memory while you are not using it; on a normal Linux PC it starts at every boot (the usual setting);
#    5. adds your user to the "docker" group (so you can run docker without sudo);
#    6. starts Docker, runs the tiny "hello-world" test container and prints where the images are stored.
#  Safe to run again: finished steps are skipped. It never touches Docker Desktop on Windows; do NOT turn on
#  Docker Desktop's "WSL integration" for this distro (two Dockers in one distro fight over the `docker` command).
#
#  HOW TO RUN (it needs admin rights):
#    from Windows PowerShell (no Ubuntu password needed):
#        wsl -d Ubuntu-22.04 -u root -- bash /mnt/d/<path to the repo>/simulation/docker/install_docker_engine.sh --user <your Ubuntu user name>
#    or inside Ubuntu:   sudo bash simulation/docker/install_docker_engine.sh
#  Options:   --user NAME      who gets docker rights (default: the sudo user, else WSL's default user)
#             --no-nvidia      skip the NVIDIA Container Toolkit   |  --nvidia  install it even if no GPU is seen
#             --start-at-boot  start Docker at every start of WSL too (instead of on first use)
#             --dry-run        only print what would be done; change nothing
#  AFTERWARDS (once): close every Ubuntu window and run  wsl --terminate Ubuntu-22.04  in PowerShell (or restart
#  the PC) so the new "docker" group applies to you. Until then use:  sg docker -c "docker ps"
#  Check:  docker run --rm hello-world        GPU check:  docker run --rm --gpus all ubuntu:22.04 nvidia-smi
# =============================================================================
set -eo pipefail

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '\033[1;32m OK %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m !! %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m XX %s\033[0m\n' "$*" >&2; exit 1; }

DRY=0; USER_NAME=""; NVIDIA=auto; AT_BOOT=auto
while [ $# -gt 0 ]; do
  case "$1" in
    --user) shift; USER_NAME="${1:?--user needs a name}" ;;
    --no-nvidia) NVIDIA=no ;;
    --nvidia) NVIDIA=yes ;;
    --start-at-boot) AT_BOOT=yes ;;
    --dry-run) DRY=1 ;;
    -h|--help) awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done

# run CMD...   : run a command, or only print it with --dry-run.
run() {
  if [ "$DRY" = 1 ]; then printf '    + %s\n' "$*"; else "$@"; fi
}
# apt_get ARGS : apt-get that waits up to 10 minutes for another apt to finish instead of failing, never asks
# questions, never restarts running services by itself (needrestart), and runs at low priority (nice).
apt_get() {
  run env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l NEEDRESTART_SUSPEND=1 \
    nice -n 10 apt-get -o DPkg::Lock::Timeout=600 "$@"
}

# ---- 0. checks -----------------------------------------------------------------------------------
[ "$DRY" = 1 ] || [ "$(id -u)" -eq 0 ] || die "This installs software, so it needs admin rights:  sudo bash $0
    (from Windows PowerShell:  wsl -d Ubuntu-22.04 -u root -- bash <this file> --user <you>)"
# shellcheck disable=SC1091
. /etc/os-release
[ "${ID:-}" = ubuntu ] || die "This script is for Ubuntu (found: ${PRETTY_NAME:-unknown})."
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
[ -n "$CODENAME" ] || die "Could not tell which Ubuntu release this is."
ARCH="$(dpkg --print-architecture)"
IS_WSL=0
grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=1
[ "$AT_BOOT" = auto ] && { if [ "$IS_WSL" = 1 ]; then AT_BOOT=no; else AT_BOOT=yes; fi; }

if [ "$(ps -p 1 -o comm= 2>/dev/null)" != systemd ]; then
  die "systemd is not running in this Ubuntu, and Docker needs it.
    WSL: add these two lines to /etc/wsl.conf, then run  wsl --shutdown  in PowerShell and open Ubuntu again:
        [boot]
        systemd=true"
fi

# Whose docker rights: --user, else the person who ran sudo, else WSL's default user (/etc/wsl.conf), else the
# first normal user (uid 1000 and up).
if [ -z "$USER_NAME" ]; then USER_NAME="${SUDO_USER:-}"; fi
if [ -z "$USER_NAME" ] && [ -f /etc/wsl.conf ]; then
  USER_NAME="$(awk -F= '/^\[user\]/{u=1; next} /^\[/{u=0} u && $1 ~ /^[ \t]*default[ \t]*$/ {gsub(/[ \t]/, "", $2); print $2}' /etc/wsl.conf)"
fi
if [ -z "$USER_NAME" ]; then USER_NAME="$(awk -F: '$3 >= 1000 && $3 < 60000 {print $1; exit}' /etc/passwd)"; fi
if [ -z "$USER_NAME" ] || ! id "$USER_NAME" >/dev/null 2>&1; then
  die "Could not find the user to give docker rights to ('${USER_NAME}'); use --user NAME."
fi

# Docker Desktop's WSL integration puts its own `docker` into this distro; two Dockers would fight.
for f in /usr/bin/docker /usr/local/bin/docker; do
  if [ -e "$f" ] && readlink -f "$f" | grep -q 'docker-desktop'; then
    die "Docker Desktop's WSL integration is switched ON for this distro ($f -> $(readlink -f "$f")).
    Turn it off first: Docker Desktop -> Settings -> Resources -> WSL integration -> untick this distro -> Apply.
    Then run this script again."
  fi
done

HAS_GPU=0
if [ -e /usr/lib/wsl/lib/libcuda.so.1 ] || { command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; }; then
  HAS_GPU=1
fi
case "$NVIDIA" in
  auto) if [ "$HAS_GPU" = 1 ]; then NVIDIA=yes; else NVIDIA=no; fi ;;
esac

say "Installing Docker Engine into $PRETTY_NAME ($ARCH)$([ "$IS_WSL" = 1 ] && echo ", inside WSL ($WSL_DISTRO_NAME)")"
info "docker rights for user:   $USER_NAME"
info "NVIDIA Container Toolkit: $NVIDIA$([ "$HAS_GPU" = 1 ] && echo " (an NVIDIA GPU is visible)")"
info "start Docker:             $([ "$AT_BOOT" = yes ] && echo "at every boot" || echo "on first use (systemd socket activation)")"
[ "$DRY" = 1 ] && info "DRY RUN: nothing is changed, the commands are only printed"

# ---- 1. Docker's apt source + packages -------------------------------------------------------------
say "[1/6] Docker's apt source and packages"
if dpkg -s docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin >/dev/null 2>&1; then
  ok "Docker Engine is already installed ($(docker --version 2>/dev/null))"
else
  apt_get update || warn "apt-get update reported a problem (often an unrelated apt source); continuing"
  apt_get install -y --no-install-recommends ca-certificates curl gnupg
  run install -m 0755 -d /etc/apt/keyrings
  run curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  run chmod a+r /etc/apt/keyrings/docker.asc
  if [ "$DRY" = 1 ]; then
    info "+ write /etc/apt/sources.list.d/docker.list"
  else
    echo "deb [arch=$ARCH signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $CODENAME stable" \
      > /etc/apt/sources.list.d/docker.list
  fi
  apt_get update || warn "apt-get update reported a problem; continuing (the install below says if Docker is missing)"
  apt_get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io docker-buildx-plugin \
    docker-compose-plugin pigz
  [ "$DRY" = 1 ] || ok "installed $(docker --version)"
fi

# ---- 2. NVIDIA Container Toolkit -------------------------------------------------------------------
say "[2/6] NVIDIA Container Toolkit (lets containers use the graphics card for CUDA)"
if [ "$NVIDIA" != yes ]; then
  info "skipped (no NVIDIA GPU seen, or --no-nvidia). Containers can still draw windows; they just get no CUDA."
elif dpkg -s nvidia-container-toolkit >/dev/null 2>&1; then
  ok "already installed ($(nvidia-ctk --version 2>/dev/null | head -1))"
else
  if [ "$DRY" = 1 ]; then
    info "+ add NVIDIA's apt source (nvidia.github.io/libnvidia-container) and its signing key"
  else
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
      | gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
      | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
      > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  fi
  apt_get update || warn "apt-get update reported a problem; continuing"
  apt_get install -y --no-install-recommends nvidia-container-toolkit
  [ "$DRY" = 1 ] || ok "installed $(nvidia-ctk --version 2>/dev/null | head -1)"
fi

# ---- 3. daemon.json ---------------------------------------------------------------------------------
say "[3/6] Docker settings (/etc/docker/daemon.json)"
# "local" log driver = compressed log files that rotate (about 100 MB per container at most). Existing settings
# in the file are kept; only missing keys are added.
if [ "$DRY" = 1 ]; then
  info "+ add {\"log-driver\": \"local\"} to /etc/docker/daemon.json (keeping what is there)"
else
  mkdir -p /etc/docker
  python3 - <<'PY'
import json, os
path = '/etc/docker/daemon.json'
cfg = {}
if os.path.exists(path):
    with open(path, encoding='utf-8') as f:
        text = f.read().strip()
    cfg = json.loads(text) if text else {}
cfg.setdefault('log-driver', 'local')
with open(path, 'w', encoding='utf-8') as f:
    json.dump(cfg, f, indent=2)
    f.write('\n')
PY
fi
if [ "$NVIDIA" = yes ]; then
  # Registers the "nvidia" runtime in daemon.json (what `--gpus all` and `--runtime nvidia` use).
  run nvidia-ctk runtime configure --runtime=docker
fi
[ "$DRY" = 1 ] || { info "now:"; sed 's/^/      /' /etc/docker/daemon.json; }

# ---- 4. when Docker starts --------------------------------------------------------------------------
say "[4/6] When Docker starts"
run systemctl daemon-reload
if [ "$AT_BOOT" = yes ]; then
  run systemctl enable docker.service containerd.service docker.socket
  info "Docker starts at every boot."
else
  # docker.socket listens on /var/run/docker.sock; the first `docker ...` command starts docker.service
  # (and containerd) automatically. Nothing runs while you do not use Docker.
  run systemctl disable docker.service containerd.service 2>/dev/null || true
  run systemctl enable docker.socket
  info "Docker starts automatically the first time you run a docker command (then it keeps running)."
fi

# ---- 5. docker group ----------------------------------------------------------------------------------
say "[5/6] Docker rights for $USER_NAME (the \"docker\" group)"
run groupadd -f docker
if id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx docker; then
  ok "$USER_NAME is already in the docker group"
else
  run usermod -aG docker "$USER_NAME"
  info "added. It applies to NEW logins: close all Ubuntu windows and run  wsl --terminate ${WSL_DISTRO_NAME:-Ubuntu-22.04}"
  info "in PowerShell (or restart the PC). Until then:  sg docker -c \"docker ps\""
fi
# Members of the docker group can do anything root can do through Docker: only add people you trust.

# ---- 6. start + test ----------------------------------------------------------------------------------
say "[6/6] Starting Docker and testing it"
if [ "$DRY" = 1 ]; then
  info "+ systemctl restart docker  (or start it)"
  info "+ docker run --rm hello-world"
  exit 0
fi
systemctl start docker.socket
if systemctl is-active --quiet docker.service; then
  systemctl restart docker.service          # picks up the new daemon.json / nvidia runtime
else
  systemctl start docker.service
fi
for _ in $(seq 1 30); do docker info >/dev/null 2>&1 && break; sleep 1; done
docker info >/dev/null 2>&1 || die "Docker did not start. See:  journalctl -u docker --no-pager | tail -40"
ok "Docker is running: $(docker version --format 'client {{.Client.Version}}, engine {{.Server.Version}}')"
if docker run --rm hello-world >/tmp/astra_hello.txt 2>&1 && grep -q 'Hello from Docker' /tmp/astra_hello.txt; then
  ok "the hello-world test container ran"
else
  warn "the hello-world test failed:"; sed 's/^/      /' /tmp/astra_hello.txt >&2
fi
rm -f /tmp/astra_hello.txt

ROOT_DIR="$(docker info --format '{{.DockerRootDir}}')"
say "Where Docker keeps images, containers and volumes"
info "$ROOT_DIR  ($(df -h "$ROOT_DIR" | awk 'NR == 2 {print $4 " free of " $2 " on " $1}'))"
if [ "$IS_WSL" = 1 ]; then
  info "That folder is inside this WSL distro's virtual disk (ext4.vhdx). Where the disk file is on Windows,"
  info "PowerShell:  (Get-ChildItem HKCU:\\Software\\Microsoft\\Windows\\CurrentVersion\\Lxss | Get-ItemProperty |"
  info "              Where-Object DistributionName -eq '${WSL_DISTRO_NAME:-Ubuntu-22.04}').BasePath"
fi
info "runtimes: $(docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}')"
WHICH="$(command -v docker)"
if [ "$WHICH" = /usr/bin/docker ]; then
  ok "\"docker\" now means Docker Engine (/usr/bin/docker), not the Windows Docker Desktop shim"
else
  warn "\"docker\" still finds $WHICH first; open a NEW terminal (old ones remember the old path) and check: type -a docker"
fi
say "DONE. Next: docs/simulation/DOCKER.md (build the images: bash simulation/docker/build_images.sh)"
[ "$NVIDIA" = yes ] && info "GPU test (after the group applies):  docker run --rm --gpus all ubuntu:22.04 nvidia-smi"
exit 0
