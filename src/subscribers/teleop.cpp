/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * LOCAL includes
 */
#include "teleop.hpp"
#include "motion_validation.hpp"
#include "helpers/async_helpers.hpp"
#include "helpers/parameter_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber(
  const std::string & name,
  const std::string & cmd_vel_topic,
  const std::string & joint_angles_topic,
  const std::string & joint_angle_traj_topic,
  const qi::SessionPtr & session)
: cmd_vel_topic_(cmd_vel_topic)
  , joint_angles_topic_(joint_angles_topic)
  , joint_angle_traj_topic_(joint_angle_traj_topic)
  , BaseSubscriber(name, cmd_vel_topic, session)
  , p_motion_(session->service("ALMotion").value())
{}

void TeleopSubscriber::reset(rclcpp::Node * node)
{
  logger_ = node->get_logger();

  const double timeout_s = helpers::declareNumberParameter(
    node, "cmd_vel_timeout", 0.5,
    "Seconds without /cmd_vel before a moving base is stopped; 0 disables");
  max_linear_speed_ = helpers::declareNumberParameter(
    node, "max_linear_speed", 0.0, "Cap on the /cmd_vel linear speed in m/s; 0 means no cap");
  max_angular_speed_ = helpers::declareNumberParameter(
    node, "max_angular_speed", 0.0, "Cap on the /cmd_vel angular speed in rad/s; 0 means no cap");
  cmd_vel_watchdog_ = std::make_shared<CmdVelWatchdog>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timeout_s)));
  cmd_vel_watchdog_timer_.reset();
  if (cmd_vel_watchdog_->enabled()) {
    cmd_vel_watchdog_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100), [this]()
      {
        if (cmd_vel_watchdog_->expired(CmdVelWatchdog::Clock::now())) {
          RCLCPP_WARN(logger_, "No /cmd_vel for the configured timeout, stopping the base");
          helpers::warnOnAsyncError(
            p_motion_.async<void>("move", 0.0f, 0.0f, 0.0f), logger_, "ALMotion.move (stop)");
        }
      });
  }
  RCLCPP_INFO(
    logger_, "cmd_vel_timeout: %.2f s%s", timeout_s,
    cmd_vel_watchdog_->enabled() ? "" : " (disabled)");

  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10,
    std::bind(&TeleopSubscriber::cmdVelCallback, this, std::placeholders::_1));

  sub_joint_angles_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
    joint_angles_topic_, 10,
    std::bind(&TeleopSubscriber::jointAnglesCallback, this, std::placeholders::_1));

  sub_joint_angle_traj_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesTrajectory>(
    joint_angle_traj_topic_, 10,
    std::bind(&TeleopSubscriber::jointAngleTrajCallback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void TeleopSubscriber::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  const std::string error = validation::validateVelocity(
    twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring /cmd_vel: %s", error.c_str());
    return;
  }

  const auto limited = validation::limitVelocity(
    twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z,
    max_linear_speed_, max_angular_speed_);
  const float vel_x = limited.x;
  const float vel_y = limited.y;
  const float vel_th = limited.theta;

  RCLCPP_DEBUG(logger_, "going to move x: %f y: %f th: %f", vel_x, vel_y, vel_th);
  if (cmd_vel_watchdog_) {
    cmd_vel_watchdog_->onCommand(
      vel_x == 0.f && vel_y == 0.f && vel_th == 0.f,
      CmdVelWatchdog::Clock::now());
  }
  helpers::warnOnAsyncError(
    p_motion_.async<void>("move", vel_x, vel_y, vel_th), logger_, "ALMotion.move");
}

void TeleopSubscriber::jointAnglesCallback(
  const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg)
{
  const std::string error = validation::validateJointAngles(
    js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring JointAnglesWithSpeed: %s", error.c_str());
    return;
  }

  if (js_msg->relative == 0) {
    helpers::warnOnAsyncError(
      p_motion_.async<void>("setAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed),
      logger_, "ALMotion.setAngles");
  } else {
    helpers::warnOnAsyncError(
      p_motion_.async<void>(
        "changeAngles", js_msg->joint_names, js_msg->joint_angles,
        js_msg->speed),
      logger_, "ALMotion.changeAngles");
  }
}

void TeleopSubscriber::jointAngleTrajCallback(
  const naoqi_bridge_msgs::msg::JointAnglesTrajectory::SharedPtr traj_msg)
{
  // Basic validation
  const auto & names = traj_msg->joint_names;
  const auto & angles = traj_msg->joint_angles;
  const auto & times = traj_msg->times;

  if (names.empty() || angles.empty() || times.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: empty fields (names:%zu angles:%zu times:%zu)",
      names.size(), angles.size(), times.size());
    return;
  }

  if (angles.size() != times.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: angles.size() (%zu) != times.size() (%zu)",
      angles.size(), times.size());
    return;
  }

  bool is_absolute = (traj_msg->relative == 0);
  bool use_bezier = traj_msg->use_bezier;  // NEW: Check for Bezier flag

  const std::string error = validation::validateTrajectory(
    names, angles, times, is_absolute,
    use_bezier);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring JointAnglesTrajectory: %s", error.c_str());
    return;
  }

  try {
    if (use_bezier) {
      // Use Bezier interpolation for smooth biological motion
      executeBezierTrajectory(names, angles, times);
    } else {
      // Original angleInterpolation logic
      executeAngleInterpolation(names, angles, times, is_absolute);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeleopSubscriber"),
      "ALMotion trajectory error: %s", e.what());
  }
}

void TeleopSubscriber::executeAngleInterpolation(
  const std::vector<std::string> & names,
  const std::vector<float> & angles,
  const std::vector<float> & times,
  bool is_absolute)
{
  if (names.size() == 1) {
    // Single joint/chain with multiple waypoints
    helpers::warnOnAsyncError(
      p_motion_.async<void>("angleInterpolation", names[0], angles, times, is_absolute), logger_,
      "ALMotion.angleInterpolation");
  } else if (names.size() == angles.size()) {
    // One waypoint per joint -> all reach their target at the same per-joint time
    std::vector<std::vector<float>> angleLists;
    std::vector<std::vector<float>> timeLists;
    angleLists.reserve(names.size());
    timeLists.reserve(names.size());

    for (size_t i = 0; i < names.size(); ++i) {
      angleLists.push_back(std::vector<float>{angles[i]});
      timeLists.push_back(std::vector<float>{times[i]});
    }
    helpers::warnOnAsyncError(
      p_motion_.async<void>(
        "angleInterpolation", names, angleLists, timeLists,
        is_absolute), logger_,
      "ALMotion.angleInterpolation");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: unsupported combination: names:%zu angles:%zu times:%zu",
      names.size(), angles.size(), times.size());
  }
}

void TeleopSubscriber::executeBezierTrajectory(
  const std::vector<std::string> & names,
  const std::vector<float> & angles,
  const std::vector<float> & times)
{
  // angleInterpolationBezier expects:
  // - jointNames: vector<string>
  // - timeLists: list of lists (ragged array)
  // - angleLists: list of lists (ragged array)

  // Determine waypoints per joint
  size_t waypoints_per_joint = angles.size() / names.size();

  if (angles.size() % names.size() != 0 || times.size() != angles.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "Bezier: invalid dimensions - names:%zu angles:%zu times:%zu",
      names.size(), angles.size(), times.size());
    return;
  }

  // Reshape flat arrays into ragged arrays
  std::vector<std::vector<float>> timeLists;
  std::vector<std::vector<float>> angleLists;
  timeLists.reserve(names.size());
  angleLists.reserve(names.size());

  for (size_t i = 0; i < names.size(); ++i) {
    std::vector<float> joint_angles;
    std::vector<float> joint_times;
    joint_angles.reserve(waypoints_per_joint);
    joint_times.reserve(waypoints_per_joint);

    for (size_t wp = 0; wp < waypoints_per_joint; ++wp) {
      joint_angles.push_back(angles[i * waypoints_per_joint + wp]);
      joint_times.push_back(times[i * waypoints_per_joint + wp]);
    }

    angleLists.push_back(joint_angles);
    timeLists.push_back(joint_times);
  }

  helpers::warnOnAsyncError(
    p_motion_.async<void>("angleInterpolationBezier", names, timeLists, angleLists), logger_,
    "ALMotion.angleInterpolationBezier");

  RCLCPP_INFO(
    rclcpp::get_logger("TeleopSubscriber"),
    "Bezier trajectory: %zu joint%s with %zu waypoint%s each",
    names.size(), names.size() == 1 ? "" : "s",
    waypoints_per_joint, waypoints_per_joint == 1 ? "" : "s");
}

}  // namespace subscriber
}  // namespace naoqi
