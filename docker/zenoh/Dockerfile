# Lightweight Zenoh router — no sensor/nav/perception payload.
# Just enough to run rmw_zenohd (same binary the rmw uses, so the
# protocol version always matches the other containers).
FROM ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY docker/zenoh/zenohd_config.json5 /etc/zenoh/zenohd_config.json5

# Standalone router binary; needs no ROS env sourcing.
CMD ["/opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd", "--cfg", "/etc/zenoh/zenohd_config.json5"]
