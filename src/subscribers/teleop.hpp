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
#ifndef TELEOP_SUBSCRIBER_HPP
#define TELEOP_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"
#include "cmd_vel_watchdog.hpp"

/*
 * ROS includes
 */
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <naoqi_bridge_msgs/msg/joint_angles_with_speed.hpp>
#include <naoqi_bridge_msgs/msg/joint_angles_trajectory.hpp>

namespace naoqi
{
namespace subscriber
{

class TeleopSubscriber : public BaseSubscriber<TeleopSubscriber>
{
public:
  TeleopSubscriber(
    const std::string & name,
    const std::string & cmd_vel_topic,
    const std::string & joint_angles_topic,
    const std::string & joint_angle_traj_topic,
    const qi::SessionPtr & session);
  ~TeleopSubscriber() override = default;

  void reset(rclcpp::Node * node) override;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void jointAnglesCallback(const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg);
  void jointAngleTrajCallback(
    const naoqi_bridge_msgs::msg::JointAnglesTrajectory::SharedPtr traj_msg);

private:
  // Helper methods for trajectory execution
  void executeAngleInterpolation(
    const std::vector<std::string> & names,
    const std::vector<float> & angles,
    const std::vector<float> & times,
    bool is_absolute);

  void executeBezierTrajectory(
    const std::vector<std::string> & names,
    const std::vector<float> & angles,
    const std::vector<float> & times);

  // Member variables
  std::string cmd_vel_topic_;
  std::string joint_angles_topic_;
  std::string joint_angle_traj_topic_;

  qi::AnyObject p_motion_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  std::shared_ptr<CmdVelWatchdog> cmd_vel_watchdog_;
  rclcpp::TimerBase::SharedPtr cmd_vel_watchdog_timer_;
  rclcpp::Logger logger_ = rclcpp::get_logger("TeleopSubscriber");
  double max_linear_speed_ = 0.0;  // m/s, 0 = no cap
  double max_angular_speed_ = 0.0;  // rad/s, 0 = no cap
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>::SharedPtr sub_joint_angles_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesTrajectory>::SharedPtr
    sub_joint_angle_traj_;
};

}  // namespace subscriber
}  // namespace naoqi
#endif
