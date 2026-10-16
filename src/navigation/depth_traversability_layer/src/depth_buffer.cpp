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

#include "depth_traversability_layer/depth_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include "tf2/time.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace depth_traversability_layer
{

void TemporalObservationQueue::push(const TileObservation & obs, bool dominant_priority)
{
  auto & queue = class_queues_[obs.class_id];
  queue.push_back(obs);
  class_confidence_sums_[obs.class_id] += obs.confidence;
  if (dominant_priority) {
    setDominant(obs.class_id, queue.size());
  } else if (prefer_max_class_id_) {
    if (dominant_class_id_ < 0 || obs.class_id > dominant_class_id_) {
      setDominant(obs.class_id, class_queues_[obs.class_id].size());
    }
  } else if (queue.size() > dominant_class_size_) {
    setDominant(obs.class_id, queue.size());
  }
}

void TemporalObservationQueue::purgeOldObservations(double current_time)
{
  for (auto it = class_queues_.begin(); it != class_queues_.end();) {
    auto & queue = it->second;
    while (!queue.empty() && (current_time - queue.front().timestamp) > decay_time_) {
      class_confidence_sums_[it->first] -= queue.front().confidence;
      queue.pop_front();
    }
    if (queue.empty()) {
      class_confidence_sums_.erase(it->first);
      it = class_queues_.erase(it);
    } else {
      ++it;
    }
  }
  recomputeDominant();
}

bool TemporalObservationQueue::empty() const
{
  return class_queues_.empty();
}

float TemporalObservationQueue::getConfidenceSum() const
{
  if (dominant_class_id_ < 0) {
    return 0.0f;
  }
  const auto it = class_confidence_sums_.find(static_cast<uint8_t>(dominant_class_id_));
  return it == class_confidence_sums_.end() ? 0.0f : it->second;
}

void TemporalObservationQueue::recomputeDominant()
{
  dominant_class_id_ = -1;
  dominant_class_size_ = 0;
  for (const auto & pair : class_queues_) {
    const int id = static_cast<int>(pair.first);
    const size_t size = pair.second.size();
    if (prefer_max_class_id_) {
      if (dominant_class_id_ < 0 || id > dominant_class_id_) {
        setDominant(id, size);
      }
    } else if (size > dominant_class_size_) {
      setDominant(id, size);
    }
  }
}

void TemporalObservationQueue::setDominant(int class_id, size_t size)
{
  dominant_class_id_ = class_id;
  dominant_class_size_ = size;
}

DepthTileMap::DepthTileMap(float resolution, double decay_time)
: resolution_(resolution), decay_time_(decay_time)
{
  // 10k observations seemed to be a good estimate for a few seconds of data.
  tile_map_.reserve(10000);
}

void DepthTileMap::clear()
{
  tile_map_.clear();
}

TileIndex DepthTileMap::worldToIndex(double x, double y) const
{
  const int ix = static_cast<int>(std::floor(x / resolution_));
  const int iy = static_cast<int>(std::floor(y / resolution_));
  return TileIndex{ix, iy};
}

TileWorldXY DepthTileMap::indexToWorld(int x, int y) const
{
  const double x_world = (static_cast<double>(x) + 0.5) * resolution_;
  const double y_world = (static_cast<double>(y) + 0.5) * resolution_;
  return TileWorldXY{x_world, y_world};
}

void DepthTileMap::pushObservation(
  const TileObservation & obs, const TileIndex & idx, bool dominant_priority)
{
  auto it = tile_map_.find(idx);
  if (it != tile_map_.end()) {
    it->second.push(obs, dominant_priority);
    return;
  }
  TemporalObservationQueue queue(decay_time_);
  queue.prefer_max_class_id_ = prefer_max_class_id_;
  queue.push(obs, dominant_priority);
  tile_map_.emplace(idx, std::move(queue));
}

void DepthTileMap::purgeOldObservations(double current_time)
{
  for (auto it = tile_map_.begin(); it != tile_map_.end();) {
    it->second.purgeOldObservations(current_time);
    if (it->second.empty()) {
      it = tile_map_.erase(it);
    } else {
      ++it;
    }
  }
}

void CostMapper::setDirectCostMapping(double scale, double offset, int ignore_lo, int ignore_hi)
{
  std::lock_guard<std::mutex> lock(mutex_);
  direct_ = true;
  scale_ = scale;
  offset_ = offset;
  ignore_lo_ = ignore_lo;
  ignore_hi_ = ignore_hi;
  ranges_.clear();
  // One "class" per possible cost (0..254), each mapping to itself, so the
  // tile / cost machinery works unchanged.
  id_to_cost_.clear();
  for (int i = 0; i <= 254; ++i) {
    const uint8_t id = static_cast<uint8_t>(i);
    id_to_cost_[id] = CostHeuristicParams{id, id, 0.0f, 0, false, 0};
  }
}

void CostMapper::setScoreMapping(const std::vector<ScoreRange> & ranges,
  const std::unordered_map<uint8_t, CostHeuristicParams> & id_to_cost)
{
  std::lock_guard<std::mutex> lock(mutex_);
  direct_ = false;
  ranges_ = ranges;
  id_to_cost_ = id_to_cost;
}

int CostMapper::resolveClassId(uint8_t pixel_value) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (direct_) {
    if (ignore_lo_ >= 0 && ignore_hi_ >= ignore_lo_ &&
      pixel_value >= ignore_lo_ && pixel_value <= ignore_hi_)
    {
      return -1;  // unknown band -> drop
    }
    const double cost = scale_ * (1.0 - static_cast<double>(pixel_value) / 255.0) + offset_;
    const int c = static_cast<int>(std::lround(cost));
    return std::max(0, std::min(254, c));
  }
  // Score mask: first range that contains the value wins.
  for (const auto & range : ranges_) {
    if (pixel_value >= range.lo && pixel_value <= range.hi) {
      return static_cast<int>(range.class_id);
    }
  }
  // No ranges configured: identity for known class ids.
  if (ranges_.empty() && id_to_cost_.count(pixel_value) > 0) {
    return static_cast<int>(pixel_value);
  }
  return -1;  // value falls in a gap -> ignore
}

CostHeuristicParams CostMapper::getCostById(uint8_t id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = id_to_cost_.find(id);
  if (it == id_to_cost_.end()) {
    return CostHeuristicParams{0, 0, 0.0f, 0, false, 0};
  }
  return it->second;
}

bool decodeDepthPixel(const sensor_msgs::msg::Image & depth, int u, int v, double & depth_m)
{
  const std::string & enc = depth.encoding;
  const bool is_16uc1 = (enc == "16UC1" || enc == "mono16");
  const bool is_32fc1 = (enc == "32FC1");
  if (!is_16uc1 && !is_32fc1) {
    return false;
  }
  const size_t elem = is_16uc1 ? 2u : 4u;
  const size_t row_step = depth.step != 0 ? static_cast<size_t>(depth.step) :
    static_cast<size_t>(depth.width) * elem;
  const size_t off = static_cast<size_t>(v) * row_step + static_cast<size_t>(u) * elem;
  if (off + elem > depth.data.size()) {
    return false;
  }
  if (is_16uc1) {
    // Little-endian millimetres; 0 means no return.
    const uint16_t raw = static_cast<uint16_t>(depth.data[off]) |
      (static_cast<uint16_t>(depth.data[off + 1]) << 8);
    if (raw == 0) {
      return false;
    }
    depth_m = static_cast<double>(raw) / 1000.0;
    return true;
  }
  float raw = 0.0f;
  std::memcpy(&raw, &depth.data[off], sizeof(float));
  if (!std::isfinite(raw) || raw <= 0.0f) {
    return false;
  }
  depth_m = static_cast<double>(raw);
  return true;
}

DepthBuffer::DepthBuffer(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
  tf2_ros::Buffer * tf, const std::string & global_frame)
: logger_(logger), clock_(std::move(clock)), tf_(tf), global_frame_(global_frame)
{
}

void DepthBuffer::configure(const DepthSourceConfig & config, float costmap_resolution)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  config_ = config;
  tile_map_ = std::make_shared<DepthTileMap>(costmap_resolution, config_.decay_time);
  if (config_.direct_cost) {
    cost_mapper_.setDirectCostMapping(config_.direct_cost_scale, config_.direct_cost_offset,
      config_.direct_cost_ignore_min, config_.direct_cost_ignore_max);
    tile_map_->setPreferMaxClassId(true);
    RCLCPP_INFO(logger_,
      "DepthBuffer: direct-cost mode ON (cost = clamp(%.1f*(1-v/255)+%.1f, 0, 254))",
      config_.direct_cost_scale, config_.direct_cost_offset);
  } else {
    std::vector<ScoreRange> ranges;
    std::unordered_map<uint8_t, CostHeuristicParams> id_to_cost;
    for (const auto & cls : config_.score_classes) {
      if (cls.value_min < 0 || cls.value_max < 0 || cls.value_min > cls.value_max ||
        cls.value_max > 255)
      {
        RCLCPP_WARN(logger_, "DepthBuffer: ignoring invalid score range for class '%s'",
          cls.name.c_str());
        continue;
      }
      ranges.push_back(ScoreRange{cls.class_id, cls.class_id, cls.class_id});
      ranges.back().lo = static_cast<uint8_t>(cls.value_min);
      ranges.back().hi = static_cast<uint8_t>(cls.value_max);
      id_to_cost[cls.class_id] = CostHeuristicParams{cls.base_cost, cls.max_cost, 0.0f,
        cls.samples_to_max_cost, cls.dominant_priority, cls.min_points_per_tile};
    }
    cost_mapper_.setScoreMapping(ranges, id_to_cost);
    RCLCPP_INFO(logger_, "DepthBuffer: score mode ON with %zu ranges", ranges.size());
  }
}

void DepthBuffer::setCameraInfo(const sensor_msgs::msg::CameraInfo & info)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (info.width == 0 || info.height == 0 || info.k[0] <= 0.0) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
      "DepthBuffer: received an unusable CameraInfo (zero size / intrinsics). Ignoring.");
    return;
  }
  fx_ = info.k[0];
  fy_ = info.k[4];
  cx_ = info.k[2];
  cy_ = info.k[5];
  cam_width_ = info.width;
  cam_height_ = info.height;
  camera_frame_ = info.header.frame_id;
  have_camera_info_ = true;
}

void DepthBuffer::bufferDepth(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & segmentation)
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (!have_camera_info_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "DepthBuffer: no usable CameraInfo received yet, cannot back-project. Skipping frame.");
    return;
  }
  const int width = static_cast<int>(depth.width);
  const int height = static_cast<int>(depth.height);
  if (width <= 0 || height <= 0 ||
    segmentation.width != depth.width || segmentation.height != depth.height)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "DepthBuffer: depth (%ux%u) != mask (%ux%u). Frames must be registered. Skipping.",
      depth.width, depth.height, segmentation.width, segmentation.height);
    return;
  }
  if (cam_width_ != depth.width || cam_height_ != depth.height) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
      "DepthBuffer: CameraInfo size (%ux%u) != depth size (%ux%u). Skipping frame.",
      cam_width_, cam_height_, depth.width, depth.height);
    return;
  }
  if (segmentation.data.size() < static_cast<size_t>(width) * static_cast<size_t>(height)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DepthBuffer: mask data too small. Skipping.");
    return;
  }
  if (depth.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DepthBuffer: depth has no frame_id. Skipping.");
    return;
  }

  const double stamp = rclcpp::Time(depth.header.stamp).seconds();
  Eigen::Affine3d depth_to_global;
  Eigen::Vector3d sensor_origin;
  try {
    const auto tf_stamped = tf_->lookupTransform(global_frame_, depth.header.frame_id,
      tf2::TimePointZero, tf2::durationFromSec(config_.transform_tolerance));
    depth_to_global = tf2::transformToEigen(tf_stamped);
    sensor_origin = depth_to_global.translation();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "DepthBuffer: TF error (%s). Skipping frame.",
      ex.what());
    return;
  }

  const double sq_min = config_.min_obstacle_distance * config_.min_obstacle_distance;
  const double sq_max = config_.max_obstacle_distance * config_.max_obstacle_distance;
  const int stride = std::max(1, config_.pixel_stride);

  // Per-tile votes this frame: class -> {count, sample u, sample v}.
  struct Vote { int count = 0; int u = 0; int v = 0; };
  std::unordered_map<TileIndex, std::unordered_map<int, Vote>> candidates;

  for (int v = 0; v < height; v += stride) {
    for (int u = 0; u < width; u += stride) {
      double d_m = 0.0;
      if (!decodeDepthPixel(depth, u, v, d_m)) {
        continue;
      }
      double ox = 0.0, oy = 0.0, oz = 0.0;
      if (!backProject(static_cast<double>(u), static_cast<double>(v), d_m, fx_, fy_, cx_, cy_,
        ox, oy, oz))
      {
        continue;
      }
      const Eigen::Vector3d pg = depth_to_global * Eigen::Vector3d(ox, oy, oz);
      if (!pg.allFinite()) {
        continue;
      }
      const double sq_dist = (pg - sensor_origin).squaredNorm();
      if (sq_dist >= sq_max || sq_dist <= sq_min) {
        continue;
      }
      const size_t mask_idx = static_cast<size_t>(v) * static_cast<size_t>(width) +
        static_cast<size_t>(u);
      const int class_id = cost_mapper_.resolveClassId(segmentation.data[mask_idx]);
      if (class_id < 0) {
        continue;
      }
      const TileIndex tile = tile_map_->worldToIndex(pg.x(), pg.y());
      auto & vote = candidates[tile][class_id];
      if (vote.count == 0) {
        vote.u = u;
        vote.v = v;
      }
      ++vote.count;
    }
  }

  // Winner per tile: highest max_cost, ties broken by vote count. All classes
  // here passed min_points_per_tile implicitly via the count gate below.
  tile_map_->purgeOldObservations(stamp);
  for (const auto & tile_entry : candidates) {
    int best_class = -1;
    uint8_t best_max_cost = 0;
    int best_count = 0;
    for (const auto & class_entry : tile_entry.second) {
      const CostHeuristicParams params =
        cost_mapper_.getCostById(static_cast<uint8_t>(class_entry.first));
      if (class_entry.second.count < params.min_points_per_tile) {
        continue;  // too few pixels of this class on the tile this frame
      }
      if (best_class < 0 || params.max_cost > best_max_cost ||
        (params.max_cost == best_max_cost && class_entry.second.count > best_count))
      {
        best_class = class_entry.first;
        best_max_cost = params.max_cost;
        best_count = class_entry.second.count;
      }
    }
    if (best_class < 0) {
      continue;
    }
    const CostHeuristicParams params =
      cost_mapper_.getCostById(static_cast<uint8_t>(best_class));
    const TileObservation obs{static_cast<uint8_t>(best_class), 1.0f, stamp};
    tile_map_->pushObservation(obs, tile_entry.first, params.dominant_priority);
  }
  last_frame_time_ = stamp;
}

void DepthBuffer::reset()
{
  std::lock_guard<std::recursive_mutex> lock(lock_);
  if (tile_map_) {
    tile_map_->clear();
  }
  last_frame_time_ = 0.0;
}

CostHeuristicParams DepthBuffer::getCostForClassId(uint8_t class_id)
{
  return cost_mapper_.getCostById(class_id);
}

}  // namespace depth_traversability_layer
