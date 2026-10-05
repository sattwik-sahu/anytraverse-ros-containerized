FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Base build tools + Spinnaker runtime deps + Ouster build deps.
# Ouster needs PCL/Eigen/jsoncpp/tins/zip; rosdep resolves most of the ROS
# deps, these are the native ones that must exist before rosdep/colcon run.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    curl \
    ca-certificates \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libomp-dev \
    libyaml-cpp-dev \
    ffmpeg \
    ethtool \
    iproute2 \
    iputils-ping \
    net-tools \
    libjsoncpp-dev \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    libtins-dev \
    libzip-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    libssl-dev \
    libspdlog-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Zenoh RMW (replaces DDS discovery; no multicast leaks between the
# switch LAN and the WiFi/base-station network). zenohd ships inside
# rmw_zenoh_cpp on Jazzy.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

# Install the Spinnaker SDK from the vendored .debs.
# Build context MUST be the workspace root so utils/ and src/ are visible.
COPY utils/spinnaker /tmp/spinnaker
# FLIR .debs gate on an interactive EULA prompt; pre-accept it so the
# non-interactive docker build does not leave packages unconfigured.
# (Analytics consent defaults to false, which is what we want.)
RUN echo "libspinnaker libspinnaker/accepted-flir-eula boolean true" | debconf-set-selections && \
    cd /tmp/spinnaker && \
    dpkg -i libgentl_*.deb || true && \
    dpkg -i libspinnaker_*.deb || true && \
    dpkg -i libspinnaker-dev_*.deb || true && \
    dpkg -i libspinnaker-c_*.deb || true && \
    dpkg -i libspinnaker-c-dev_*.deb || true && \
    dpkg -i libspinvideo_*.deb || true && \
    dpkg -i libspinvideo-dev_*.deb || true && \
    dpkg -i libspinvideo-c_*.deb || true && \
    dpkg -i libspinvideo-c-dev_*.deb || true && \
    dpkg -i spinupdate_*.deb || true && \
    dpkg -i spinupdate-dev_*.deb || true && \
    dpkg -i spinnaker_*.deb || true && \
    dpkg -i spinnaker-doc_*.deb || true && \
    apt-get update && apt-get install -f -y && \
    test -d /opt/spinnaker/lib || (echo "Spinnaker SDK install failed" >&2; exit 1) && \
    ldconfig && \
    rm -rf /tmp/spinnaker /var/lib/apt/lists/*

# NOTE: spinview-qt is GUI-only; skip in this headless image.
# For a debug image add:
#   dpkg -i spinview-qt_*.deb spinview-qt-dev_*.deb

WORKDIR /ws
COPY src/sensors/ ./src/sensors/

# Explicitly install the known ROS deps first so the image does not depend
# on rosdep resolving every third-party package (e.g. rko_lio is an
# exec_depend of sensors_bringup but is not in this repo).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-camera-info-manager \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-image-transport \
    ros-jazzy-rclcpp-components \
    ros-jazzy-rclcpp-lifecycle \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-proc \
    ros-jazzy-image-geometry \
    ros-jazzy-camera-calibration-parsers \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-ament-cmake-auto \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/* \
    && (rosdep update || true) \
    && (rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true)

# Build the sensor stack. This pulls in flir_camera_msgs,
# spinnaker_camera_driver (+synchronized), ouster_sensor_msgs, ouster_ros,
# image_proc (from src/sensors/image_pipeline) and sensors_bringup.
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      --packages-up-to sensors_bringup

COPY docker/sensors/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
