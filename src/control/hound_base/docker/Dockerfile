FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# hound_base is pure Python (rclpy): only serial/numpy beyond ros-base,
# plus the Zenoh RMW for the shared-router pattern.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-numpy \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

WORKDIR /ws
COPY src/control/hound_base/ ./src/hound_base/

RUN (rosdep update || true) \
    && (rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true)

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-select hound_base

COPY docker/controller/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Default: drive base from /cmd_vel with the in-package params.
# The workspace compose overrides params_file with the robot template.
CMD ["ros2", "launch", "hound_base", "base.launch.py"]
