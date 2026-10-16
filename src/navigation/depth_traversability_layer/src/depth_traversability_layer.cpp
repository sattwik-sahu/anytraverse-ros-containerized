// Copyright 2026 AnyTraverse contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "depth_traversability_layer/depth_traversability_layer.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rmw/qos_profiles.h"

namespace depth_traversability_layer
{

void DepthTraversabilityLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("combination_method", rclcpp::ParameterValue(1));
  declareParameter("depth_topic", rclcpp::ParameterValue(std::string("depth")));
  declareParameter("camera_info_topic", rclcpp::ParameterValue(std::string("camera_info")));
  declareParameter("segmentation_topic", rclcpp::ParameterValue(std::string("segmentation")));
  declareParameter("approximate_sync", rclcpp::ParameterValue(true));
  declareParameter("approximate_sync_tolerance", rclcpp::ParameterValue(0.05));
  declareParameter("pixel_stride", rclcpp::ParameterValue(2));
  declareParameter("min_obstacle_distance", rclcpp::ParameterValue(0.3));
  declareParameter("max_obstacle_distance", rclcpp::ParameterValue(20.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declareParameter("expected_update_rate", rclcpp::ParameterValue(10.0));
  declareParameter("tile_map_decay_time", rclcpp::ParameterValue(3.0));
  declareParameter("direct_cost", rclcpp::ParameterValue(true));
  declareParameter("direct_cost_scale", rclcpp::ParameterValue(254.0));
  declareParameter("direct_cost_offset", rclcpp::ParameterValue(0.0));
  declareParameter("direct_cost_ignore_min", rclcpp::ParameterValue(-1));
  declareParameter("direct_cost_ignore_max", rclcpp::ParameterValue(-1));

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "DepthTraversabilityLayer: failed to lock node in onInitialize");
    return;
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "combination_method", combination_method_);
  rolling_window_ = layered_costmap_->isRolling();

  DepthSourceConfig config;
  std::string depth_topic, info_topic, mask_topic;
  node->get_parameter(name_ + ".depth_topic", depth_topic);
  node->get_parameter(name_ + ".camera_info_topic", info_topic);
  node->get_parameter(name_ + ".segmentation_topic", mask_topic);
  bool approximate_sync = true;
  double sync_tolerance = 0.05;
  node->get_parameter(name_ + ".approximate_sync", approximate_sync);
  node->get_parameter(name_ + ".approximate_sync_tolerance", sync_tolerance);
  node->get_parameter(name_ + ".pixel_stride", config.pixel_stride);
  node->get_parameter(name_ + ".min_obstacle_distance", config.min_obstacle_distance);
  node->get_parameter(name_ + ".max_obstacle_distance", config.max_obstacle_distance);
  node->get_parameter(name_ + ".transform_tolerance", config.transform_tolerance);
  node->get_parameter(name_ + ".expected_update_rate", expected_update_rate_);
  node->get_parameter(name_ + ".tile_map_decay_time", config.decay_time);
  node->get_parameter(name_ + ".direct_cost", config.direct_cost);
  node->get_parameter(name_ + ".direct_cost_scale", config.direct_cost_scale);
  node->get_parameter(name_ + ".direct_cost_offset", config.direct_cost_offset);
  node->get_parameter(name_ + ".direct_cost_ignore_min", config.direct_cost_ignore_min);
  node->get_parameter(name_ + ".direct_cost_ignore_max", config.direct_cost_ignore_max);

  if (!config.direct_cost) {
    // Score mode: one param block per class name listed in `classes`.
    std::vector<std::string> class_names;
    declareParameter("classes", rclcpp::ParameterValue(std::vector<std::string>()));
    node->get_parameter(name_ + ".classes", class_names);
    for (const auto & cls : class_names) {
      DepthSourceConfig::ScoreClass sc;
      sc.name = cls;
      int class_id = 0, lo = -1, hi = -1, base = 0, maxi = 0, samples = 0, min_points = 1;
      declareParameter(cls + ".class_id", rclcpp::ParameterValue(0));
      declareParameter(cls + ".value_min", rclcpp::ParameterValue(-1));
      declareParameter(cls + ".value_max", rclcpp::ParameterValue(-1));
      declareParameter(cls + ".base_cost", rclcpp::ParameterValue(0));
      declareParameter(cls + ".max_cost", rclcpp::ParameterValue(0));
      declareParameter(cls + ".samples_to_max_cost", rclcpp::ParameterValue(0));
      declareParameter(cls + ".min_points_per_tile", rclcpp::ParameterValue(1));
      declareParameter(cls + ".dominant_priority", rclcpp::ParameterValue(false));
      node->get_parameter(name_ + "." + cls + ".class_id", class_id);
      node->get_parameter(name_ + "." + cls + ".value_min", lo);
      node->get_parameter(name_ + "." + cls + ".value_max", hi);
      node->get_parameter(name_ + "." + cls + ".base_cost", base);
      node->get_parameter(name_ + "." + cls + ".max_cost", maxi);
      node->get_parameter(name_ + "." + cls + ".samples_to_max_cost", samples);
      node->get_parameter(name_ + "." + cls + ".min_points_per_tile", min_points);
      node->get_parameter(name_ + "." + cls + ".dominant_priority", sc.dominant_priority);
      sc.class_id = static_cast<uint8_t>(std::max(0, std::min(255, class_id)));
      sc.value_min = lo;
      sc.value_max = hi;
      sc.base_cost = static_cast<uint8_t>(std::max(0, std::min(254, base)));
      sc.max_cost = static_cast<uint8_t>(std::max(0, std::min(254, maxi)));
      sc.samples_to_max_cost = samples;
      sc.min_points_per_tile = min_points;
      config.score_classes.push_back(sc);
    }
  }

  buffer_ = std::make_shared<DepthBuffer>(logger_, node->get_clock(), tf_,
    layered_costmap_->getGlobalFrameID());
  buffer_->configure(config, layered_costmap_->getCostmap()->getResolution());

  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  depth_sub_ = std::make_shared<message_filters::Subscriber<
    sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(
    node, depth_topic, rmw_qos_profile_sensor_data);
  mask_sub_ = std::make_shared<message_filters::Subscriber<
    sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(
    node, mask_topic, rmw_qos_profile_sensor_data);
  auto cb = std::bind(&DepthTraversabilityLayer::depthMaskCb, this,
    std::placeholders::_1, std::placeholders::_2);
  if (approximate_sync) {
    approx_sync_ = std::make_shared<message_filters::Synchronizer<ApproxDepthMaskPolicy>>(
      ApproxDepthMaskPolicy(50), *depth_sub_, *mask_sub_);
    approx_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance));
    approx_sync_->registerCallback(cb);
  } else {
    exact_sync_ = std::make_shared<message_filters::Synchronizer<ExactDepthMaskPolicy>>(
      ExactDepthMaskPolicy(50), *depth_sub_, *mask_sub_);
    exact_sync_->registerCallback(cb);
  }
  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, sensor_qos,
    std::bind(&DepthTraversabilityLayer::cameraInfoCb, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "DepthTraversabilityLayer [%s]: depth=%s mask=%s info=%s stride=%d",
    name_.c_str(), depth_topic.c_str(), mask_topic.c_str(), info_topic.c_str(),
    config.pixel_stride);
  current_ = true;
}

void DepthTraversabilityLayer::depthMaskCb(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth,
  const sensor_msgs::msg::Image::ConstSharedPtr & segmentation)
{
  if (!enabled_ || !buffer_) {
    return;
  }
  buffer_->bufferDepth(*depth, *segmentation);
}

void DepthTraversabilityLayer::cameraInfoCb(
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
  if (!enabled_ || !buffer_) {
    return;
  }
  buffer_->setCameraInfo(*info);
}

void DepthTraversabilityLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_ || !buffer_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "DepthTraversabilityLayer: failed to lock node in updateBounds");
    return;
  }
  const double current_time = node->now().seconds();
  if (current_time <= 0.0) {
    RCLCPP_WARN(logger_, "DepthTraversabilityLayer: invalid current time in updateBounds");
    return;
  }
  const double last_frame = buffer_->lastFrameTime();
  if (expected_update_rate_ > 0.0 && last_frame > 0.0 &&
    (current_time - last_frame) > 3.0 / expected_update_rate_)
  {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 5000,
      "DepthTraversabilityLayer [%s]: no depth frames for %.1fs (expected %.1f Hz)",
      name_.c_str(), current_time - last_frame, expected_update_rate_);
  }

  // Tiles written this cycle; compared with marked_tiles_ below to clear decayed ones.
  std::unordered_set<TileIndex> current_tiles;

  buffer_->lock();
  buffer_->getTileMap()->purgeOldObservations(current_time);
  for (const auto & tile_entry : *buffer_->getTileMap()) {
    if (tile_entry.second.empty()) {
      continue;
    }
    const TileWorldXY world = buffer_->getTileMap()->indexToWorld(
      tile_entry.first.x, tile_entry.first.y);
    const TemporalObservationQueue & queue = tile_entry.second;
    unsigned int mx = 0, my = 0;
    if (!worldToMap(world.x, world.y, mx, my)) {
      RCLCPP_DEBUG(logger_, "DepthTraversabilityLayer: tile outside costmap");
      continue;
    }
    const CostHeuristicParams params =
      buffer_->getCostForClassId(static_cast<uint8_t>(queue.getClassId()));
    const size_t size = queue.size();
    if (size >= static_cast<size_t>(std::max(0, params.samples_to_max_cost)) &&
      (size == 0 || queue.getConfidenceSum() / static_cast<float>(size) >
        params.mark_confidence))
    {
      setCost(mx, my, params.max_cost);
    } else {
      setCost(mx, my, params.base_cost);
    }
    touch(world.x, world.y, min_x, min_y, max_x, max_y);
    current_tiles.insert(tile_entry.first);
  }
  buffer_->unlock();

  // Clear what decayed: tiles written last cycle whose observations have all expired
  // (purgeOldObservations erased them) go back to free, and their bounds are
  // touched so the master costmap is reset and recomputed there.
  for (const auto & tile : marked_tiles_) {
    if (current_tiles.count(tile) > 0) {
      continue;
    }
    const TileWorldXY world = buffer_->getTileMap()->indexToWorld(tile.x, tile.y);
    unsigned int mx = 0, my = 0;
    if (worldToMap(world.x, world.y, mx, my)) {  // still inside the (rolling) window
      setCost(mx, my, 0);
      touch(world.x, world.y, min_x, min_y, max_x, max_y);
    }
  }
  marked_tiles_ = std::move(current_tiles);
}

void DepthTraversabilityLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i,
  int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void DepthTraversabilityLayer::reset()
{
  resetMaps();
  marked_tiles_.clear();
  current_ = false;
  was_reset_ = true;
  // Unlike the reference layer this also clears the temporal observations, so
  // a reset (e.g. clear-entirely recovery) cannot redraw stale tiles.
  if (buffer_) {
    buffer_->reset();
  }
}

}  // namespace depth_traversability_layer

PLUGINLIB_EXPORT_CLASS(
  depth_traversability_layer::DepthTraversabilityLayer, nav2_costmap_2d::Layer)
