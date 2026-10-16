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
//
// Depth-image counterpart of semantic_segmentation_layer's tile machinery:
// back-projected depth pixels vote per costmap tile, observations carry
// timestamps, old ones are purged by decay time, dominance is highest cost
// (worst / safest).

#ifndef DEPTH_TRAVERSABILITY_LAYER__DEPTH_BUFFER_HPP_
#define DEPTH_TRAVERSABILITY_LAYER__DEPTH_BUFFER_HPP_

#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"

namespace depth_traversability_layer
{

/// @brief 2D grid index with equality comparison. Supports negative indexes.
struct TileIndex
{
  int x, y;

  bool operator==(const TileIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

}  // namespace depth_traversability_layer

namespace std
{
// Declared immediately after TileIndex: unordered_map/set members below
// instantiate it, and a later specialization would be ill-formed.
template<>
struct hash<depth_traversability_layer::TileIndex>
{
  size_t operator()(const depth_traversability_layer::TileIndex & coord) const
  {
    return std::hash<int>()(coord.x) ^ (std::hash<int>()(coord.y) << 1);
  }
};
}  // namespace std

namespace depth_traversability_layer
{

/// @brief World (x, y) position, typically a tile center.
struct TileWorldXY
{
  double x, y;
};

/// @brief Cost parameters attached to one score class (or one direct-cost value).
struct CostHeuristicParams
{
  uint8_t base_cost = 0;
  uint8_t max_cost = 0;
  float mark_confidence = 0.0f;
  int samples_to_max_cost = 0;
  bool dominant_priority = false;
  // Observations of this class landing on one tile in one frame before the
  // class can claim the tile. 1 = any single observation counts.
  int min_points_per_tile = 1;
};

/// @brief Maps a range of raw mono8 mask values to a class id, for masks that
/// carry a continuous score (e.g. traversability 0..255) instead of discrete ids.
struct ScoreRange
{
  uint8_t lo, hi;
  uint8_t class_id;
};

/// @brief One buffered per-tile observation.
struct TileObservation
{
  uint8_t class_id;
  float confidence;
  double timestamp;
};

/// @brief Per-tile observation history with decay. Dominant class is the
/// largest queue, or the highest class id when prefer-max mode is on
/// (direct-cost mode, where the id IS the cost).
class TemporalObservationQueue
{
public:
  explicit TemporalObservationQueue(double decay_time) : decay_time_(decay_time) {}

  void push(const TileObservation & obs, bool dominant_priority = false);
  /// @brief Drop observations older than decay_time_ seconds.
  void purgeOldObservations(double current_time);
  bool empty() const;
  /// @brief Number of observations in the dominant class.
  size_t size() const { return dominant_class_size_; }
  int getClassId() const { return dominant_class_id_; }
  /// @brief Summed confidence of the dominant class (for the mark gate).
  float getConfidenceSum() const;

private:
  void recomputeDominant();
  void setDominant(int class_id, size_t size);

  std::unordered_map<uint8_t, std::deque<TileObservation>> class_queues_;
  std::unordered_map<uint8_t, float> class_confidence_sums_;
  int dominant_class_id_ = -1;
  size_t dominant_class_size_ = 0;
  double decay_time_;
  bool prefer_max_class_id_ = false;

  friend class DepthTileMap;
};

/// @brief World-anchored map of tile observations with a decay mechanism.
class DepthTileMap
{
public:
  using Iterator = typename std::unordered_map<TileIndex, TemporalObservationQueue>::iterator;
  using ConstIterator =
    typename std::unordered_map<TileIndex, TemporalObservationQueue>::const_iterator;
  using SharedPtr = std::shared_ptr<DepthTileMap>;

  DepthTileMap(float resolution, double decay_time);
  DepthTileMap() = default;

  Iterator begin() { return tile_map_.begin(); }
  Iterator end() { return tile_map_.end(); }
  void lock() { lock_.lock(); }
  void unlock() { lock_.unlock(); }
  size_t size() { return tile_map_.size(); }
  void clear();

  /// @brief World coordinates to tile index (floor, same as reference layer).
  TileIndex worldToIndex(double x, double y) const;
  /// @brief Tile index to world coordinates of the tile center.
  TileWorldXY indexToWorld(int x, int y) const;

  void pushObservation(const TileObservation & obs, const TileIndex & idx,
    bool dominant_priority = false);
  void purgeOldObservations(double current_time);
  void setPreferMaxClassId(bool prefer) { prefer_max_class_id_ = prefer; }

private:
  std::unordered_map<TileIndex, TemporalObservationQueue> tile_map_;
  float resolution_ = 0.05f;
  double decay_time_ = 5.0;
  std::recursive_mutex lock_;
  bool prefer_max_class_id_ = false;
};

/// @brief Resolves raw mono8 mask bytes to class ids / costs.
/// Direct-cost mode: cost = clamp(scale*(1-v/255)+offset, 0, 254), so a
/// 0=blocked / 255=free map needs no inversion.
/// Score mode: first [lo, hi] range containing the value wins; gaps are dropped.
class CostMapper
{
public:
  void setDirectCostMapping(double scale, double offset, int ignore_lo, int ignore_hi);
  void setScoreMapping(const std::vector<ScoreRange> & ranges,
    const std::unordered_map<uint8_t, CostHeuristicParams> & id_to_cost);
  /// @return class id (direct mode: the cost itself), or -1 to drop the pixel.
  int resolveClassId(uint8_t pixel_value) const;
  CostHeuristicParams getCostById(uint8_t id) const;

private:
  bool direct_ = false;
  double scale_ = 254.0, offset_ = 0.0;
  int ignore_lo_ = -1, ignore_hi_ = -1;
  std::vector<ScoreRange> ranges_;
  std::unordered_map<uint8_t, CostHeuristicParams> id_to_cost_;
  mutable std::mutex mutex_;
};

/// @brief Back-project a depth pixel into the optical frame.
/// @return false for non-positive depth.
inline bool backProject(double u, double v, double depth_m, double fx, double fy, double cx,
  double cy, double & x, double & y, double & z)
{
  if (!(depth_m > 0.0))
  {
    return false;
  }
  x = (u - cx) * depth_m / fx;
  y = (v - cy) * depth_m / fy;
  z = depth_m;
  return std::isfinite(x) && std::isfinite(y);
}

/// @brief Decode one depth pixel to meters. Supports 16UC1 (mm) and 32FC1 (m).
/// @return false for invalid pixels (zero / NaN / Inf / non-positive).
bool decodeDepthPixel(const sensor_msgs::msg::Image & depth, int u, int v, double & depth_m);

/// @brief All tunable parameters of one depth source (flat namespace).
struct DepthSourceConfig
{
  double min_obstacle_distance = 0.3;
  double max_obstacle_distance = 20.0;
  double transform_tolerance = 0.1;
  double expected_update_rate = 10.0;
  double decay_time = 3.0;
  int pixel_stride = 2;
  bool direct_cost = true;
  double direct_cost_scale = 254.0;
  double direct_cost_offset = 0.0;
  int direct_cost_ignore_min = -1;
  int direct_cost_ignore_max = -1;
  struct ScoreClass
  {
    std::string name;
    uint8_t class_id = 0;
    int value_min = -1, value_max = -1;
    uint8_t base_cost = 0, max_cost = 0;
    int samples_to_max_cost = 0, min_points_per_tile = 1;
    bool dominant_priority = false;
  };
  std::vector<ScoreClass> score_classes;
};

/// @brief Consumes synchronized (depth, mask) frames: back-projects, votes per
/// tile, commits timestamped observations. CameraInfo cached separately.
class DepthBuffer
{
public:
  using SharedPtr = std::shared_ptr<DepthBuffer>;

  DepthBuffer(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
    tf2_ros::Buffer * tf, const std::string & global_frame);

  void configure(const DepthSourceConfig & config, float costmap_resolution);
  void setCameraInfo(const sensor_msgs::msg::CameraInfo & info);
  void bufferDepth(const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::Image & segmentation);
  void reset();
  void lock() { tile_map_->lock(); }
  void unlock() { tile_map_->unlock(); }
  DepthTileMap::SharedPtr getTileMap() { return tile_map_; }
  CostHeuristicParams getCostForClassId(uint8_t class_id);
  double lastFrameTime() const { return last_frame_time_; }

private:
  void commitObservations(
    const std::unordered_map<TileIndex, std::pair<int, size_t>> & winners,
    const sensor_msgs::msg::Image & segmentation, double stamp_seconds);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer * tf_;
  std::string global_frame_;

  DepthSourceConfig config_;
  DepthTileMap::SharedPtr tile_map_;
  CostMapper cost_mapper_;

  // Cached intrinsics.
  bool have_camera_info_ = false;
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  uint32_t cam_width_ = 0, cam_height_ = 0;
  std::string camera_frame_;

  double last_frame_time_ = 0.0;
  std::recursive_mutex lock_;
};

}  // namespace depth_traversability_layer

#endif  // DEPTH_TRAVERSABILITY_LAYER__DEPTH_BUFFER_HPP_
