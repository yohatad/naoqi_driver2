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

namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber(const std::string& name,
                                   const std::string& cmd_vel_topic,
                                   const std::string& joint_angles_topic,
                                   const std::string& joint_angle_traj_topic,
                                   const qi::SessionPtr& session)
: cmd_vel_topic_(cmd_vel_topic)
, joint_angles_topic_(joint_angles_topic)
, joint_angle_traj_topic_(joint_angle_traj_topic)
, BaseSubscriber(name, cmd_vel_topic, session)
, p_motion_(session->service("ALMotion").value())
{}

void TeleopSubscriber::reset(rclcpp::Node* node)
{
  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10,
      std::bind(&TeleopSubscriber::cmd_vel_callback, this, std::placeholders::_1));

  sub_joint_angles_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
      joint_angles_topic_, 10,
      std::bind(&TeleopSubscriber::joint_angles_callback, this, std::placeholders::_1));

  sub_joint_angle_traj_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesTrajectory>(
      joint_angle_traj_topic_, 10,
      std::bind(&TeleopSubscriber::joint_angle_traj_callback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void TeleopSubscriber::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  const float& vel_x  = twist_msg->linear.x;
  const float& vel_y  = twist_msg->linear.y;
  const float& vel_th = twist_msg->angular.z;

  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  p_motion_.async<void>("move", vel_x, vel_y, vel_th);
}

void TeleopSubscriber::joint_angles_callback(
    const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg)
{
  if (js_msg->relative == 0)
  {
    p_motion_.async<void>("setAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  }
  else
  {
    p_motion_.async<void>("changeAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  }
}

void TeleopSubscriber::joint_angle_traj_callback(
    const naoqi_bridge_msgs::msg::JointAnglesTrajectory::SharedPtr traj_msg)
{
  // Basic validation
  const auto& names  = traj_msg->joint_names;
  const auto& angles = traj_msg->joint_angles;
  const auto& times  = traj_msg->times;

  if (names.empty() || angles.empty() || times.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopSubscriber"),
                "JointAnglesTrajectory: empty fields (names:%zu angles:%zu times:%zu)",
                names.size(), angles.size(), times.size());
    return;
  }

  if (angles.size() != times.size())
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopSubscriber"),
                "JointAnglesTrajectory: angles.size() (%zu) != times.size() (%zu)",
                angles.size(), times.size());
    return;
  }

  bool is_absolute = (traj_msg->relative == 0);
  bool use_bezier = traj_msg->use_bezier;  // NEW: Check for Bezier flag

  try
  {
    if (use_bezier)
    {
      // Use Bezier interpolation for smooth biological motion
      execute_bezier_trajectory(names, angles, times);
    }
    else
    {
      // Original angleInterpolation logic
      execute_angle_interpolation(names, angles, times, is_absolute);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TeleopSubscriber"),
                 "ALMotion trajectory error: %s", e.what());
  }
}

void TeleopSubscriber::execute_angle_interpolation(
    const std::vector<std::string>& names,
    const std::vector<float>& angles,
    const std::vector<float>& times,
    bool is_absolute)
{
  if (names.size() == 1)
  {
    // Single joint/chain with multiple waypoints
    p_motion_.async<void>("angleInterpolation", names[0], angles, times, is_absolute);
  }
  else if (names.size() == angles.size())
  {
      // One waypoint per joint -> all reach their target at the same per-joint time
      std::vector<std::vector<float>> angleLists;
      std::vector<std::vector<float>> timeLists;
      angleLists.reserve(names.size());
      timeLists.reserve(names.size());
      
      for (size_t i = 0; i < names.size(); ++i)
      {
        angleLists.push_back(std::vector<float>{angles[i]});
        timeLists.push_back(std::vector<float>{times[i]});
      }
      p_motion_.async<void>("angleInterpolation", names, angleLists, timeLists, is_absolute);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopSubscriber"),
                "JointAnglesTrajectory: unsupported combination: names:%zu angles:%zu times:%zu",
                names.size(), angles.size(), times.size());
  }
}

void TeleopSubscriber::execute_bezier_trajectory(
    const std::vector<std::string>& names,
    const std::vector<float>& angles,
    const std::vector<float>& times)
{
  // angleInterpolationBezier expects:
  // - jointNames: vector<string>
  // - timeLists: list of lists (ragged array)
  // - angleLists: list of lists (ragged array)
  
  // Determine waypoints per joint
  size_t waypoints_per_joint = angles.size() / names.size();
  
  if (angles.size() % names.size() != 0 || times.size() != angles.size())
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopSubscriber"),
                "Bezier: invalid dimensions - names:%zu angles:%zu times:%zu",
                names.size(), angles.size(), times.size());
    return;
  }
  
  // Reshape flat arrays into ragged arrays
  std::vector<std::vector<float>> timeLists;
  std::vector<std::vector<float>> angleLists;
  timeLists.reserve(names.size());
  angleLists.reserve(names.size());
  
  for (size_t i = 0; i < names.size(); ++i)
  {
    std::vector<float> joint_angles;
    std::vector<float> joint_times;
    joint_angles.reserve(waypoints_per_joint);
    joint_times.reserve(waypoints_per_joint);
    
    for (size_t wp = 0; wp < waypoints_per_joint; ++wp)
    {
      joint_angles.push_back(angles[i * waypoints_per_joint + wp]);
      joint_times.push_back(times[i * waypoints_per_joint + wp]);
    }
    
    angleLists.push_back(joint_angles);
    timeLists.push_back(joint_times);
  }
  
  p_motion_.async<void>("angleInterpolationBezier", names, timeLists, angleLists);
  
  RCLCPP_INFO(rclcpp::get_logger("TeleopSubscriber"),
              "Bezier trajectory: %zu joint%s with %zu waypoint%s each",
              names.size(), names.size() == 1 ? "" : "s",
              waypoints_per_joint, waypoints_per_joint == 1 ? "" : "s");
}

} // namespace subscriber
} // namespace naoqi
