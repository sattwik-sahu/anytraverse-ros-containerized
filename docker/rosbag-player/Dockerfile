FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-default-plugins \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "entrypoint"]
CMD ["ros2", "bag", "play", "/bags/rosbag2_2026_10_20-12_53_56", "--clock", "--rate", "1.0", \
     "--topics", "/tf", "/tf_static", "/odom", "/lidar/points", "/lidar/points_lio", \
     "/lidar/imu", "/camera/rgb/image_raw", "/camera/rgb/camera_info", \
     "/segmentation_mask/image_raw", "/segmentation_mask/camera_info", \
     "/flir_camera/meta", "/rko_lio/local_map"]
