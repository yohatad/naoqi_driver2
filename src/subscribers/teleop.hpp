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
#include "proximity_guard.hpp"

/*
 * ROS includes
 */
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <naoqi_bridge_msgs/msg/bumper.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
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

  // Coarse laser/sonar/bumper backstop; see proximity_guard.hpp for why it
  // lives here rather than in Nav2's collision monitor.
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void bumperCallback(const naoqi_bridge_msgs::msg::Bumper::SharedPtr msg);

private:
  /// Worst level across the fresh sources, plus the bumper clamp.
  safety::Clamp currentClamp();
  /// Called on a timer: ALMotion.move() latches its last velocity, so a new
  /// detection has to push a zero even when /cmd_vel has gone quiet.
  void proximityTick();
  void publishProximityDiagnostics(safety::Level level, const safety::Clamp & clamp);

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
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Logger logger_ = rclcpp::get_logger("TeleopSubscriber");
  double max_linear_speed_ = 0.0;  // m/s, 0 = no cap
  double max_angular_speed_ = 0.0;  // rad/s, 0 = no cap

  // --- proximity backstop ---
  bool proximity_enabled_ = false;
  safety::Zones zones_;
  double source_timeout_ = 1.0;         // s before a reading stops counting
  std::shared_ptr<safety::BumperLatch> bumper_latch_;
  safety::SonarFilter sonar_filter_{2};
  safety::Level laser_level_ = safety::Level::kClear;
  safety::Level sonar_level_ = safety::Level::kClear;
  double laser_stamp_ = -1.0;
  double sonar_stamp_ = -1.0;
  bool moving_ = false;                 // last command was non-zero
  bool stop_sent_ = false;              // zero already pushed for this detection
  std::mutex proximity_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sonar_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::Bumper>::SharedPtr sub_bumper_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr proximity_timer_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>::SharedPtr sub_joint_angles_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesTrajectory>::SharedPtr
    sub_joint_angle_traj_;
};

}  // namespace subscriber
}  // namespace naoqi
#endif
