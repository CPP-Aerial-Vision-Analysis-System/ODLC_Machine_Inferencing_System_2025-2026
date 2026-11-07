#!/bin/bash
# /opt/astra/in_stack.sh -- PURPOSE: run one command inside the Jetson-in-the-loop container with ROS 2 Humble and
# the team's workspace sourced, the same ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY as the running stack, in the repo folder.
# `jetson.sh exec -- CMD` and `jetson.sh shell` call it as the stack's own user (astra-dev): a ROS command run as
# root would not see the team stack (docs/jetson/HIL_DESIGN.md 8.6). The exit code is CMD's.
# Usage: /opt/astra/in_stack.sh CMD [ARGS...]      (no CMD: an interactive bash)
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source /home/astra-dev/astra/ros2_ws/install/setup.bash
# take the stack's ROS settings from PID 1's environment when this shell does not have them (docker exec starts clean)
if [ -z "${ROS_DOMAIN_ID:-}" ] && [ -r /proc/1/environ ]; then
  eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ASTRA_RUN_ID|ASTRA_PIPELINE)=' | sed 's/^/export /')"
fi
cd /home/astra-dev/astra 2>/dev/null || true
[ $# -gt 0 ] || set -- bash
exec "$@"
