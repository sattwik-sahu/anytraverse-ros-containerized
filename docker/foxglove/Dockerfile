FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

WORKDIR /ws
# Custom message typesupport needed to subscribe/plot custom topics
# (anytraverse_msgs/State, Roi, ...). Standard types (Image, PointCloud2,
# TF, Odometry, ...) come from ros-base. Pure-IDL package, builds in seconds.
COPY src/perception/anytraverse_ros/src/anytraverse_msgs/ ./src/anytraverse_msgs/

RUN (rosdep update || true) \
    && (rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true)

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-select anytraverse_msgs

COPY docker/foxglove/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Per Foxglove docs, launch via the provided launch file: the node binary
# takes no --port/--address CLI flags (params go through launch args).
# Port defaults to 8765 and address to 0.0.0.0 in the package itself, so
# Studio connects to ws://<JETSON_WIFI_IP>:8765 (allow 8765/tcp on the host).
# The bridge subscribes upstream lazily: only topics opened in Studio cost
# CPU/bandwidth, so idle cost is ~zero. Prefer /compressed topics over WiFi.
CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", "port:=8765"]
