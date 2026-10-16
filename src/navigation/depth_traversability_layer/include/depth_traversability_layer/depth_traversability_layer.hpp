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

#ifndef DEPTH_TRAVERSABILITY_LAYER__DEPTH_TRAVERSABILITY_LAYER_HPP_
#define DEPTH_TRAVERSABILITY_LAYER__DEPTH_TRAVERSABILITY_LAYER_HPP_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "depth_traversability_layer/depth_buffer.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depth_traversability_layer
{

typedef message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image>
  ExactDepthMaskPolicy;
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image>
  ApproxDepthMaskPolicy;

/**
 * @brief Costmap layer fusing a registered depth image with a traversability
 * mask. Mirrors semantic_segmentation_layer's architecture (temporal tile map
 * with decay, per-tile voting, max/overwrite combine) with the single change
 * that 3D points come from depth back-projection instead of a point cloud.
 */
class DepthTraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DepthTraversabilityLayer() = default;
  ~DepthTraversabilityLayer() override = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void depthMaskCb(const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::Image::ConstSharedPtr & segmentation);
  void cameraInfoCb(sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

  DepthBuffer::SharedPtr buffer_;
  bool rolling_window_ = true;

  // Subscriptions must stay alive as members. CameraInfo arrives latched and
  // is cached; (depth, mask) arrive through an exact or approximate synchronizer.
  std::shared_ptr<message_filters::Subscriber<
    sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<
    sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> mask_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactDepthMaskPolicy>> exact_sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproxDepthMaskPolicy>> approx_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  std::unordered_set<TileIndex> marked_tiles_;
  bool current_ = true;
  bool was_reset_ = false;
  int combination_method_ = 1;
  double expected_update_rate_ = 10.0;
};

}  // namespace depth_traversability_layer

#endif  // DEPTH_TRAVERSABILITY_LAYER__DEPTH_TRAVERSABILITY_LAYER_HPP_
