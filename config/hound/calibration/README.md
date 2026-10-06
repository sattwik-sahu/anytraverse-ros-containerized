# Camera calibration files for this robot go here.
#
# Convention: the calibration lives in this dir as `camera_params.yaml`.
# The compose file mounts it into the sensors container at
# /root/.ros/camera_info/flir_camera.yaml — the exact path the FLIR driver
# (via camera_info_manager) loads for the `flir_camera` node. If you rename
# the camera node, update the target side of that mount to <name>.yaml.
#
# The file already present was produced by camera calibration
# (checkerboard -> ost.yaml format: camera_matrix, distortion_*,
# rectification_matrix, projection_matrix for 1440x1080).
#
# How to (re)calibrate (checkerboard, from the Jetson):
#   1. Print a checkerboard (e.g. 9x6, 25mm squares) and mount it rigidly.
#   2. Run the calibrator against the live raw topic:
#        docker compose run --rm sensors ros2 run camera_calibration cameracalibrator \
#          --size 9x6 --square 0.025 \
#          --image:=/flir_camera/image_raw --camera:=/flir_camera
#      (cover the full field of view: tilt/translate the board, watch the bars go green)
#   3. Click CALIBRATE, then SAVE. Overwrite this file with the result:
#        cp /tmp/ost.yaml config/hound/calibration/camera_params.yaml
#   4. docker compose restart sensors   (no rebuild needed)
#
# Verify: `ros2 topic echo /flir_camera/camera_info -n1` should show your
# camera_matrix (K[0] != 0) instead of all zeros.
