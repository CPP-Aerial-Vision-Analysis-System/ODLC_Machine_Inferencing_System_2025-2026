#!/bin/bash
# =============================================================================
#  simulation/docker/require_image.sh  --  stop with a CLEAR message when a local ASTRA image is not built yet
#
#  PURPOSE: the images astra-dev, astra-sitl and astra-sim exist only on a PC where somebody built them
#  (build_images.sh) or loaded them (docker load). They are NOT on Docker Hub. When a tool such as VS Code's
#  "Reopen in Container" asks Docker for a missing one, Docker looks for it on Docker Hub and answers
#      pull access denied ... repository does not exist or may require 'docker login'
#  which sends people looking for a login that is not needed. This script is run first (by
#  .devcontainer/sim/devcontainer.json "initializeCommand") and explains the real situation instead.
#
#  USAGE:  bash simulation/docker/require_image.sh IMAGE TARGET        e.g.  astra-sim:latest sim
#          exit 0 = the image exists on this PC;  exit 1 = it does not (the message says what to do).
#  Needs only the `docker` command. Changes nothing.
# =============================================================================
img="${1:?usage: require_image.sh IMAGE TARGET   (for example: astra-sim:latest sim)}"
target="${2:?usage: require_image.sh IMAGE TARGET   (for example: astra-sim:latest sim)}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not available in this terminal. Install Docker Engine inside Ubuntu (WSL) or on your Linux PC:" >&2
  echo "    sudo bash simulation/docker/install_docker_engine.sh        (see docs/simulation/DOCKER.md, step A3)" >&2
  exit 1
fi

# Exit 0 quietly when the image is there. A Docker that is not running also fails this test: say so separately.
if docker image inspect "$img" >/dev/null 2>&1; then exit 0; fi
if ! docker info >/dev/null 2>&1; then
  echo "Docker is installed but not running (or you may not be in the 'docker' group yet)." >&2
  echo "    Engine in WSL/Linux:  sudo systemctl start docker     Docker Desktop: start the app." >&2
  echo "    'permission denied': log out and back in once after installing (docs/simulation/DOCKER.md)." >&2
  exit 1
fi

cat >&2 <<EOF

=============================================================================
 The Docker image "$img" has not been built on this PC yet.

 This is NOT a login problem. No Docker account is needed. The ASTRA images are built on your own computer;
 they are not published on Docker Hub, so Docker would otherwise say
 "pull access denied ... repository does not exist or may require 'docker login'".

 Build it once, in an Ubuntu terminal (WSL on Windows), from the repository folder:

     bash simulation/docker/build_images.sh $target

 The first build takes about an hour and uses about 25 GB of disk; later builds reuse what is unchanged.
 Or load images a teammate saved: see "Copy images without rebuilding" in docs/simulation/DOCKER.md.
 Then open the dev container again.
=============================================================================
EOF
exit 1
