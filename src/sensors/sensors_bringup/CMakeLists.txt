cmake_minimum_required(VERSION 3.8)
project(sensors_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_guard.py DESTINATION lib/${PROJECT_NAME})

ament_package()
