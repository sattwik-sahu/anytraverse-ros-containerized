# Lightweight Zenoh router — no sensor/nav/perception payload.
# Just enough to run rmw_zenohd (same binary the rmw uses, so the
# protocol version always matches the other containers).
FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    iproute2 \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY docker/zenoh/zenohd_config.json5 /etc/zenoh/zenohd_config.json5
COPY docker/zenoh/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Do not rely on the base image's LD_LIBRARY_PATH (it changed across
# upstream rebuilds); the entrypoint sources /opt/ros setup explicitly.
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib

# Standalone router binary; needs no ROS env sourcing.
ENTRYPOINT ["/entrypoint.sh"]
