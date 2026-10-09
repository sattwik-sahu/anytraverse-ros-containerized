#!/bin/bash
set -e
# Zenoh defaults; override per-environment via docker compose / -e.
: "${RMW_IMPLEMENTATION:=rmw_zenoh_cpp}"
: "${ROS_DOMAIN_ID:=0}"
export RMW_IMPLEMENTATION ROS_DOMAIN_ID
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
