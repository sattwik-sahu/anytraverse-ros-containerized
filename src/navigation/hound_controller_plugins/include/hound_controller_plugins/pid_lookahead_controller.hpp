// PID look-ahead controller for Nav2: steers with a PID on the heading error to a
// look-ahead point on the path, under Ackermann limits (never turns in place).
#ifndef HOUND_CONTROLLER_PLUGINS__PID_LOOKAHEAD_CONTROLLER_HPP_
#define HOUND_CONTROLLER_PLUGINS__PID_LOOKAHEAD_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hound_controller_plugins
{

/**
 * Each cycle:
 *  1. find the path pose closest to the robot (searching forward from the start of
 *     the latest plan), then the look-ahead point: the first point on the path at
 *     look-ahead distance L = clamp(v * lookahead_time, min, max) from the robot;
 *  2. tracking error = heading error to that point, atan2(y, x) in the robot frame;
 *  3. angular.z = PID(error), with conditional integration (no windup) and a
 *     low-pass filtered derivative;
 *  4. linear.x = desired_linear_vel, scaled down by cos(error) and near the goal,
 *     never below min_linear_vel (the motors stall below it) and capped by any
 *     Nav2 speed limit;
 *  5. Ackermann: |angular.z| <= linear.x / min_turning_radius -- the robot always
 *     moves forward while turning and never turns tighter than that radius.
 * Stopping at the goal is left to Nav2's goal checker, as with any controller.
 */
class PidLookaheadController : public nav2_core::Controller
{
public:
  PidLookaheadController() = default;
  ~PidLookaheadController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  void resetPid();
  // Robot pose expressed in the plan's frame.
  bool robotPoseInPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::PoseStamped & out);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PidLookaheadController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  // parameters
  double desired_linear_vel_, min_linear_vel_, min_approach_linear_vel_, approach_dist_;
  double lookahead_time_, min_lookahead_dist_, max_lookahead_dist_;
  double kp_, ki_, kd_, integral_limit_, derivative_filter_;
  double max_angular_vel_, min_turning_radius_, max_robot_pose_search_dist_;
  double transform_tolerance_;

  // state
  std::mutex mutex_;
  nav_msgs::msg::Path plan_;
  size_t progress_{0};  // index of the closest path pose last cycle; reset by setPlan
  double speed_limit_;  // m/s, or < 0 = none
  double integral_{0.0}, prev_error_{0.0}, d_filtered_{0.0}, last_linear_{0.0};
  rclcpp::Time last_time_;
  bool have_last_{false};

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;
  // [error, p, i, d, angular_cmd, linear_cmd, lookahead_dist]
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
};

}  // namespace hound_controller_plugins

#endif  // HOUND_CONTROLLER_PLUGINS__PID_LOOKAHEAD_CONTROLLER_HPP_
