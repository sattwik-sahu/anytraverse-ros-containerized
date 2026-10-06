#!/bin/sh
# Prints a friendly "reachable at" banner, then hands off to the router.
# Runs on every container start, so `docker compose up zenohd` always
# shows where the router can be reached.
set -e
echo "--- zenoh router starting (rmw_zenohd) ---"
ip -4 -o addr show scope global | awk '{print $2, $4}' | while read -r iface addr; do
  case "$iface" in
    docker*|br-*|veth*|l4tbr*) continue ;;
  esac
  echo "--- zenoh router can be connected at ${addr%%/*}:7447 (via ${iface})"
done
echo "--- launching rmw_zenohd --cfg /etc/zenoh/zenohd_config.json5"
# rmw_zenohd needs the ROS library paths (base-image ENV for these is not
# reliable across upstream rebuilds, so source setup explicitly).
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.sh
exec /opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd --cfg /etc/zenoh/zenohd_config.json5
