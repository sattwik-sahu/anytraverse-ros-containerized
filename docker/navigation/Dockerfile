FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Nav2 stack + build tools + segmentation-layer deps.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-util \
    ros-jazzy-vision-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-pluginlib \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Zenoh RMW (same router pattern as the sensors container).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

WORKDIR /ws
COPY src/navigation/ ./src/navigation/

RUN (rosdep update || true) \
    && (rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true)

# Build the custom Nav2 plugins (Nav2 itself comes from apt above).
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-up-to semantic_segmentation_layer depth_traversability_layer hound_controller_plugins

COPY docker/navigation/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Default: full Nav2 navigation with the compose-mounted params file.
# Params file is bind-mounted at /etc/nav2/nav2_params.yaml by docker-compose.
CMD ["ros2", "launch", "nav2_bringup", "navigation_launch.py", \
     "params_file:=/etc/nav2/nav2_params.yaml", \
     "autostart:=True", "use_sim_time:=False"]
