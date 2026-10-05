#!/bin/bash
set -e
# Spinnaker GenTL producer location (matches configure_gentl_paths.sh 64)
export GENICAM_GENTL64_PATH=/opt/spinnaker/lib/spinnaker-gentl:${GENICAM_GENTL64_PATH}
export SPINNAKER_GENTL64_CTI=/opt/spinnaker/lib/spinnaker-gentl/Spinnaker_GenTL.cti
# Zenoh defaults; override per-environment via docker compose / -e.
: "${RMW_IMPLEMENTATION:=rmw_zenoh_cpp}"
: "${ROS_DOMAIN_ID:=0}"
export RMW_IMPLEMENTATION ROS_DOMAIN_ID
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
