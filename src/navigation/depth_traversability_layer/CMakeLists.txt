cmake_minimum_required(VERSION 3.8)
project(depth_traversability_layer)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

set(dependencies
  message_filters
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
)

include_directories(include)

# Create the depth traversability layer library
add_library(depth_traversability_layer SHARED
  src/depth_traversability_layer.cpp
  src/depth_buffer.cpp
)

target_include_directories(depth_traversability_layer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(depth_traversability_layer ${dependencies})
target_link_libraries(depth_traversability_layer Eigen3::Eigen)

# Install library
install(TARGETS depth_traversability_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description file for pluginlib
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(depth_traversability_layer)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_math test/test_depth_math.cpp)
  target_link_libraries(test_depth_math depth_traversability_layer)
  ament_target_dependencies(test_depth_math rclcpp sensor_msgs tf2)
endif()

ament_package()
