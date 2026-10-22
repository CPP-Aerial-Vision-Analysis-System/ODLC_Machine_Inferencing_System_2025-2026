#!/usr/bin/env bash
# =============================================================================
#  fetch_sitl.sh  --  get ANOTHER ArduCopter version for the simulator (run it whenever you need one)
#
#  PURPOSE: the simulator normally flies ArduCopter 4.6.3 (the version on the real Pixhawk, installed in
#  ~/sitl by 02_setup_ardupilot.sh). To try the drone's code with another firmware version (a newer release,
#  the current beta, the newest development build), this script downloads that version's ready-made SITL
#  program from ArduPilot's firmware server, with the two parameter files of the SAME version, into
#      ~/sitl/versions/<VERSION>/arducopter, copter.parm, gazebo-iris.parm, SOURCE.txt
#  The default ~/sitl/arducopter is never touched. Then fly it with:
#      bash simulation/scripts/demo.sh runway --sitl-version <VERSION>
#      bash simulation/scripts/start_sim.sh runway sitl_version:=<VERSION>
#      or in a profile:  autopilot: {version: "<VERSION>"}        (docs/simulation/ARDUPILOT.md)
#
#  HOW TO RUN:   bash fetch_sitl.sh 4.7.1          -> ArduCopter 4.7.1 (firmware.ardupilot.org/Copter/stable-4.7.1)
#                bash fetch_sitl.sh stable | beta | latest   -> the newest stable / beta / development build
#                bash fetch_sitl.sh 4.7.1 --dry-run -> print what would be downloaded and where (no internet)
#                bash fetch_sitl.sh --list          -> the versions the firmware server has (needs internet)
#  Options:      --force          download again even when that version is already there
#                --url URL        take the arducopter program from this address (e.g. a CI build) instead
#                --from-file F    copy this arducopter program (a teammate's own build) instead of downloading
#                --params-ref R   git tag or branch for copter.parm / gazebo-iris.parm (default: Copter-<VERSION>
#                                 for a number like 4.7.1, else the version found inside the program, else master)
#                --dest DIR       install into DIR instead of ~/sitl/versions/<VERSION>
#  Settings:     ASTRA_SITL_VERSIONS (default ~/sitl/versions) = the folder that holds the versions.
#  Notes:        only x86_64 Linux (a normal PC, WSL, Docker on a PC) has ready-made SITL programs. On a
#                Jetson / ARM computer build SITL from source instead (docs/simulation/ARDUPILOT.md).
#                A version name may only contain letters, digits, dots, dashes and underscores.
#  Exit codes:   0 = installed (or already there), 1 = download / check failed, 2 = bad usage.
# =============================================================================
set -o pipefail

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '\033[1;32m OK %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m !! %s\033[0m\n' "$*" >&2; }
fail() { printf '\033[1;31m XX %s\033[0m\n' "$*" >&2; exit 1; }
usage_error() { printf '\033[1;31m XX %s\033[0m\n' "$*" >&2; echo "    (see: bash $0 --help)" >&2; exit 2; }

FIRMWARE="https://firmware.ardupilot.org/Copter"
BOARD="SITL_x86_64_linux_gnu"
RAW="https://raw.githubusercontent.com/ArduPilot/ardupilot"
PARAM_FILES=(copter.parm gazebo-iris.parm)

VERSION=""; FORCE=0; DRY=0; LIST=0; URL=""; FROM_FILE=""; PARAMS_REF=""; DEST=""
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; exit 0 ;;
    --force) FORCE=1 ;;
    --dry-run) DRY=1 ;;
    --list) LIST=1 ;;
    --url) shift; URL="${1:-}"; [ -n "$URL" ] || usage_error "--url needs an address" ;;
    --from-file) shift; FROM_FILE="${1:-}"; [ -n "$FROM_FILE" ] || usage_error "--from-file needs a file" ;;
    --params-ref) shift; PARAMS_REF="${1:-}"; [ -n "$PARAMS_REF" ] || usage_error "--params-ref needs a tag or branch" ;;
    --dest) shift; DEST="${1:-}"; [ -n "$DEST" ] || usage_error "--dest needs a folder" ;;
    -*) usage_error "unknown option $1" ;;
    *) [ -z "$VERSION" ] || usage_error "only one VERSION, got \"$VERSION\" and \"$1\""; VERSION="$1" ;;
  esac
  shift
done

# ---- --list: what the firmware server offers ---------------------------------------------------------
if [ "$LIST" = 1 ]; then
  say "ArduCopter versions on $FIRMWARE (stable-X.Y.Z = a release; also: stable, beta, latest)"
  page="$(curl -fsSL --max-time 30 "$FIRMWARE/")" || fail "could not read $FIRMWARE/ (internet?)"
  printf '%s\n' "$page" | grep -oE '(stable|beta)-[0-9]+\.[0-9]+\.[0-9]+[A-Za-z0-9.-]*' | sort -u -V | sed 's/^/    /'
  exit 0
fi

# ---- the version name --------------------------------------------------------------------------------
[ -n "$VERSION" ] || usage_error "which version? e.g.  bash $0 4.7.1   (or stable, beta, latest)"
case "$VERSION" in Copter-*|copter-*|ArduCopter-*|V[0-9]*|v[0-9]*) VERSION="${VERSION#*[-Vv]}" ;; esac
[[ "$VERSION" =~ ^[0-9A-Za-z][0-9A-Za-z._-]{0,31}$ ]] || usage_error "\"$VERSION\" is not a version name like 4.7.1, stable, beta or latest"
VERSIONS_DIR="${ASTRA_SITL_VERSIONS:-$HOME/sitl/versions}"
DEST="${DEST:-$VERSIONS_DIR/$VERSION}"

# Where the program comes from: --from-file, --url, or the firmware server's folders for this version.
CANDIDATES=()
if [ -n "$FROM_FILE" ]; then
  SOURCE_KIND="file"
elif [ -n "$URL" ]; then
  SOURCE_KIND="url"; CANDIDATES=("$URL")
else
  SOURCE_KIND="firmware server"
  case "$VERSION" in
    stable|beta|latest) CANDIDATES=("$FIRMWARE/$VERSION/$BOARD/arducopter") ;;
    *) CANDIDATES=("$FIRMWARE/stable-$VERSION/$BOARD/arducopter" "$FIRMWARE/beta-$VERSION/$BOARD/arducopter") ;;
  esac
fi
# Which git tag/branch the parameter files come from (they must match the firmware version).
if [ -z "$PARAMS_REF" ]; then
  if [[ "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then PARAMS_REF="Copter-$VERSION"; else PARAMS_REF="(from the program's version, else master)"; fi
fi

say "ArduCopter SITL $VERSION -> $DEST"
info "program from: $SOURCE_KIND ${FROM_FILE:-${CANDIDATES[*]}}"
info "parameter files ${PARAM_FILES[*]} from: $RAW/$PARAMS_REF/Tools/autotest/default_params/"
if [ "$DRY" = 1 ]; then
  ok "dry run: nothing was downloaded or changed"
  exit 0
fi

ARCH="${ASTRA_FETCH_ARCH:-$(uname -m)}"
if [ -z "$FROM_FILE" ] && [ "$ARCH" != x86_64 ]; then
  fail "this computer is $ARCH: ready-made SITL programs exist only for x86_64. Build SITL from source instead
    (git clone https://github.com/ArduPilot/ardupilot, ./waf configure --board sitl && ./waf copter) and use
    --from-file <tree>/build/sitl/bin/arducopter, or the sitl_source profile setting (docs/simulation/ARDUPILOT.md)"
fi
if [ -x "$DEST/arducopter" ] && [ "$FORCE" = 0 ]; then
  ok "already there: $DEST/arducopter ($(grep -a -o -m1 -E 'ArduCopter V[0-9]+\.[0-9]+\.[0-9]+[^ ]* \([0-9a-f]+\)' "$DEST/arducopter" | head -1))"
  info "(--force downloads it again)"
  exit 0
fi
command -v curl > /dev/null || fail "curl is missing: sudo apt install curl"

# Everything goes into a temporary folder first, so a failed download never leaves half a version behind.
mkdir -p "$(dirname "$DEST")" || fail "cannot create $(dirname "$DEST")"
TMP="$(mktemp -d "$(dirname "$DEST")/.fetch_${VERSION}_XXXXXX")" || fail "cannot create a temporary folder"
trap 'rm -rf "$TMP"' EXIT

FROM=""
if [ -n "$FROM_FILE" ]; then
  [ -f "$FROM_FILE" ] || fail "file not found: $FROM_FILE"
  cp "$FROM_FILE" "$TMP/arducopter" || fail "cannot copy $FROM_FILE"
  FROM="$(readlink -f "$FROM_FILE")"
else
  for u in "${CANDIDATES[@]}"; do
    info "downloading $u"
    if curl -fL --retry 2 --connect-timeout 20 --max-time 600 -o "$TMP/arducopter" "$u"; then FROM="$u"; break; fi
  done
  [ -n "$FROM" ] || fail "ArduCopter $VERSION was not found on the firmware server (tried: ${CANDIDATES[*]}).
    See which versions exist with:  bash $0 --list"
fi

# Is it really a Linux program, and which version is compiled into it?
MAGIC="$(head -c 4 "$TMP/arducopter" | od -An -tx1 | tr -d ' \n')"
[ "$MAGIC" = "7f454c46" ] || fail "the downloaded file is not a Linux program (no ELF header): $FROM"
chmod +x "$TMP/arducopter"
VER_LINE="$(grep -a -o -m1 -E 'ArduCopter V[0-9]+\.[0-9]+\.[0-9]+[^ ]* \([0-9a-f]+\)' "$TMP/arducopter" | head -1)"
if [ -n "$VER_LINE" ]; then ok "program: $VER_LINE"; else warn "could not read the version inside the program"; fi

# Parameter files of the same version.
if [[ "$PARAMS_REF" == "("* ]]; then
  INNER="$(printf '%s' "$VER_LINE" | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
  REFS=()
  [ -z "$INNER" ] || REFS+=("Copter-$INNER")
  REFS+=(master)
else
  REFS=("$PARAMS_REF" master)
fi
PARAMS_FROM=""
for f in "${PARAM_FILES[@]}"; do
  got=0
  for ref in "${REFS[@]}"; do
    if curl -fsSL --retry 2 --connect-timeout 20 --max-time 120 -o "$TMP/$f" "$RAW/$ref/Tools/autotest/default_params/$f"; then
      got=1; PARAMS_FROM="${PARAMS_FROM:+$PARAMS_FROM, }$f@$ref"; break
    fi
  done
  if [ "$got" = 0 ]; then
    if [ -f "$HOME/sitl/$f" ]; then
      cp "$HOME/sitl/$f" "$TMP/$f"; PARAMS_FROM="${PARAMS_FROM:+$PARAMS_FROM, }$f@~/sitl (4.6.3 copy)"
      warn "$f could not be downloaded: using the copy from ~/sitl (made for 4.6.3)"
    else
      warn "$f could not be downloaded: the simulator will look for it in ~/sitl when SITL starts"
    fi
  fi
done

SHA="$(sha256sum "$TMP/arducopter" | cut -d' ' -f1)"
{
  echo "# Written by simulation/setup/fetch_sitl.sh on $(date '+%Y-%m-%d %H:%M:%S')"
  echo "version_name: $VERSION"
  echo "version_line: ${VER_LINE:-unknown}"
  echo "program_from: $FROM"
  echo "program_sha256: $SHA"
  echo "parameter_files: ${PARAMS_FROM:-none}"
} > "$TMP/SOURCE.txt"

# Swap the new folder into place (the old one, with --force, is removed only after the new one is complete).
if [ -d "$DEST" ]; then rm -rf "$DEST.old" && mv "$DEST" "$DEST.old"; fi
mv "$TMP" "$DEST" || fail "cannot move the download to $DEST"
trap - EXIT
rm -rf "$DEST.old"
ok "installed: $DEST"
ls -l "$DEST" | sed 's/^/    /'
info "fly it:  bash simulation/scripts/demo.sh runway --sitl-version $VERSION"
info "     or: bash simulation/scripts/start_sim.sh runway sitl_version:=$VERSION"
