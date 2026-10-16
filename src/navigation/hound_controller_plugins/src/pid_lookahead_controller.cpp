#include "hound_controller_plugins/pid_lookahead_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hound_controller_plugins
{

void PidLookaheadController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("PidLookaheadController: unable to lock node");
  }
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare = [&](const std::string & p, double v) {
      nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + "." + p, rclcpp::ParameterValue(v));
      double out;
      node->get_parameter(plugin_name_ + "." + p, out);
      return out;
    };
  desired_linear_vel_ = declare("desired_linear_vel", 0.8);
  min_linear_vel_ = declare("min_linear_vel", 0.4);
  min_approach_linear_vel_ = declare("min_approach_linear_vel", 0.4);
  approach_dist_ = declare("approach_dist", 1.0);
  lookahead_time_ = declare("lookahead_time", 1.5);
  min_lookahead_dist_ = declare("min_lookahead_dist", 1.5);
  max_lookahead_dist_ = declare("max_lookahead_dist", 3.0);
  kp_ = declare("kp", 1.5);
  ki_ = declare("ki", 0.0);
  kd_ = declare("kd", 0.1);
  integral_limit_ = declare("integral_limit", 0.5);
  derivative_filter_ = declare("derivative_filter", 0.7);
  max_angular_vel_ = declare("max_angular_vel", 1.0);
  min_turning_radius_ = declare("min_turning_radius", 1.5);
  max_robot_pose_search_dist_ = declare("max_robot_pose_search_dist", 3.0);
  transform_tolerance_ = declare("transform_tolerance", 0.1);
  speed_limit_ = -1.0;

  if (min_turning_radius_ <= 0.0) {
    throw nav2_core::InvalidController("PidLookaheadController: min_turning_radius must be > 0");
  }
  lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/pid_tracking", 1);

  RCLCPP_INFO(
    logger_, "PidLookaheadController '%s': kp %.2f ki %.2f kd %.2f, v %.2f (min %.2f), "
    "lookahead %.1f-%.1f m (%.1f s), Ackermann min turning radius %.2f m",
    plugin_name_.c_str(), kp_, ki_, kd_, desired_linear_vel_, min_linear_vel_,
    min_lookahead_dist_, max_lookahead_dist_, lookahead_time_, min_turning_radius_);
}

void PidLookaheadController::cleanup()
{
  lookahead_pub_.reset();
  debug_pub_.reset();
}

void PidLookaheadController::activate()
{
  lookahead_pub_->on_activate();
  debug_pub_->on_activate();
  resetPid();
}

void PidLookaheadController::deactivate()
{
  lookahead_pub_->on_deactivate();
  debug_pub_->on_deactivate();
  resetPid();
}

void PidLookaheadController::resetPid()
{
  integral_ = prev_error_ = d_filtered_ = last_linear_ = 0.0;
  have_last_ = false;
}

void PidLookaheadController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Replanning happens several times a second: keep the PID state across plans (the
  // look-ahead point barely moves), reset only when a goal starts (computeVelocity
  // does that after a gap in calls).
  plan_ = path;
  progress_ = 0;  // a new plan starts at (or near) the robot
}

void PidLookaheadController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (speed_limit <= 0.0 || (percentage && speed_limit >= 100.0)) {  // NO_SPEED_LIMIT
    speed_limit_ = -1.0;
  } else {
    speed_limit_ = percentage ? desired_linear_vel_ * speed_limit / 100.0 : speed_limit;
  }
}

bool PidLookaheadController::robotPoseInPlanFrame(
  const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::PoseStamped & out)
{
  if (pose.header.frame_id == plan_.header.frame_id) {
    out = pose;
    return true;
  }
  try {
    out = tf_->transform(pose, plan_.header.frame_id, tf2::durationFromSec(transform_tolerance_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "PidLookaheadController: robot pose -> %s: %s", plan_.header.frame_id.c_str(), ex.what());
    return false;
  }
}

geometry_msgs::msg::TwistStamped PidLookaheadController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (plan_.poses.empty()) {
    throw nav2_core::InvalidPath("PidLookaheadController: no plan");
  }
  geometry_msgs::msg::PoseStamped robot;
  if (!robotPoseInPlanFrame(pose, robot)) {
    throw nav2_core::ControllerTFError("PidLookaheadController: can't transform the robot pose");
  }
  const double rx = robot.pose.position.x, ry = robot.pose.position.y;
  const double ryaw = tf2::getYaw(robot.pose.orientation);

  // 1. closest pose, searching forward from last cycle's closest pose for at most
  //    max_robot_pose_search_dist along the path: progress is kept on a long-lived plan,
  //    and a path that loops back can't capture the robot.
  const auto & poses = plan_.poses;
  progress_ = std::min(progress_, poses.size() - 1);
  size_t closest = progress_;
  double best = std::numeric_limits<double>::max(), arc = 0.0;
  for (size_t i = progress_; i < poses.size(); ++i) {
    if (i > progress_) {
      arc += std::hypot(
        poses[i].pose.position.x - poses[i - 1].pose.position.x,
        poses[i].pose.position.y - poses[i - 1].pose.position.y);
      if (arc > max_robot_pose_search_dist_) {break;}
    }
    double d = std::hypot(poses[i].pose.position.x - rx, poses[i].pose.position.y - ry);
    if (d < best) {best = d; closest = i;}
  }
  progress_ = closest;

  //    look-ahead point: first path point at distance >= L from the robot, interpolated
  //    on the segment that crosses the circle; the goal if the path ends inside it.
  const double L = std::clamp(last_linear_ * lookahead_time_, min_lookahead_dist_, max_lookahead_dist_);
  double lx = poses.back().pose.position.x, ly = poses.back().pose.position.y;
  for (size_t i = closest + 1; i < poses.size(); ++i) {
    const auto & a = poses[i - 1].pose.position;
    const auto & b = poses[i].pose.position;
    double db = std::hypot(b.x - rx, b.y - ry);
    if (db >= L) {
      double da = std::hypot(a.x - rx, a.y - ry);
      double t = (db - da) > 1e-6 ? std::clamp((L - da) / (db - da), 0.0, 1.0) : 1.0;
      lx = a.x + t * (b.x - a.x);
      ly = a.y + t * (b.y - a.y);
      break;
    }
  }
  const double dist_to_goal = std::hypot(poses.back().pose.position.x - rx, poses.back().pose.position.y - ry);

  // 2. tracking error: heading error to the look-ahead point, robot frame.
  const double dx = lx - rx, dy = ly - ry;
  const double x_r = std::cos(ryaw) * dx + std::sin(ryaw) * dy;
  const double y_r = -std::sin(ryaw) * dx + std::cos(ryaw) * dy;
  const double error = std::atan2(y_r, x_r);

  // 3. PID. A gap in calls (> 0.5 s) means a new goal or a pause: start fresh.
  const rclcpp::Time now = clock_->now();
  double dt = have_last_ ? (now - last_time_).seconds() : 0.0;
  if (!have_last_ || dt > 0.5 || dt <= 0.0) {
    integral_ = 0.0;
    d_filtered_ = 0.0;
    prev_error_ = error;
    dt = 0.0;
  }
  double derivative = 0.0;
  if (dt > 0.0) {
    // wrap the difference so crossing +-pi doesn't spike the derivative
    double de = std::atan2(std::sin(error - prev_error_), std::cos(error - prev_error_));
    derivative = de / dt;
  }
  d_filtered_ = derivative_filter_ * d_filtered_ + (1.0 - derivative_filter_) * derivative;
  const double new_integral = std::clamp(integral_ + error * dt, -integral_limit_, integral_limit_);
  const double p = kp_ * error, d = kd_ * d_filtered_;
  double angular = p + ki_ * new_integral + d;

  // 4. speed: slow for large heading errors (cos) and near the goal, never below the
  //    motors' stall speed; cap by any Nav2 speed limit.
  double linear = desired_linear_vel_ * std::max(std::cos(error), 0.0);
  if (dist_to_goal < approach_dist_) {
    linear = std::min(linear, std::max(min_approach_linear_vel_, desired_linear_vel_ * dist_to_goal / approach_dist_));
  }
  if (speed_limit_ > 0.0) {
    linear = std::min(linear, speed_limit_);
  }
  linear = std::max(linear, min_linear_vel_);  // always moving: Ackermann can't turn standing still

  // 5. Ackermann: turn no tighter than min_turning_radius at this speed.
  const double max_w = std::min(max_angular_vel_, linear / min_turning_radius_);
  const double clamped = std::clamp(angular, -max_w, max_w);
  // Conditional integration: don't wind up the integral while saturated in its direction.
  if (clamped == angular || (angular > 0) != (error > 0)) {
    integral_ = new_integral;
  }
  angular = clamped;

  prev_error_ = error;
  last_time_ = now;
  have_last_ = true;
  last_linear_ = linear;

  if (lookahead_pub_->is_activated() && lookahead_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PointStamped pt;
    pt.header.frame_id = plan_.header.frame_id;
    pt.header.stamp = now;
    pt.point.x = lx;
    pt.point.y = ly;
    lookahead_pub_->publish(pt);
  }
  if (debug_pub_->is_activated() && debug_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {error, p, ki_ * integral_, d, angular, linear, L};
    debug_pub_->publish(dbg);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = now;
  cmd.twist.linear.x = linear;
  cmd.twist.angular.z = angular;
  return cmd;
}

}  // namespace hound_controller_plugins

PLUGINLIB_EXPORT_CLASS(hound_controller_plugins::PidLookaheadController, nav2_core::Controller)
