#!/bin/bash
# Field WASD teleop for the hound.
#
# Runs teleop_wasd inside the already-running controller container (same
# image, same middleware, shared /cmd_vel topic) with your keyboard attached
# directly — no Compose stdin multiplexing in the way.
#
# Usage: ./scripts/teleop.sh
#   Requires zenohd + controller already up. Keep Nav2 idle while driving
#   (teleop publishes the shared /cmd_vel topic). Robot on blocks first.
#   Ctrl-C quits teleop only; the driver keeps running.
set -e

if ! docker ps --format '{{.Names}}' | grep -qx 'anytraverse-controller'; then
  echo "error: anytraverse-controller is not running. Start it first, then $0." >&2
  exit 1
fi

exec docker exec -it anytraverse-controller bash -c \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run hound_base teleop_wasd --ros-args -p topic:=/hound/cmd_vel"
