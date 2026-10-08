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

#ifndef GLOBALRECORDER_HPP
#define GLOBALRECORDER_HPP

/*
* LOCAL includes
*/
#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/ros_helpers.hpp>

/*
* STANDARD includes
*/
#include <atomic>
#include <mutex>
#include <string>

/*
* ROS includes
*/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace naoqi
{
namespace recorder
{

/**
* @brief Tracks the ROS bag recording state shared by all recorders.
* @note Writing bags is not implemented for ROS 2: start/stop only track the state and
* the bag name, and write() stores nothing.
*/
class GlobalRecorder
{

public:
  /**
  * @param prefix_topic namespace prepended to relative topic names
  */
  GlobalRecorder(const std::string & prefix_topic);

  /**
  * @brief Initialize the recording of the ROSbag
  */
  void startRecord(const std::string & prefix_bag = "");

  /**
  * @brief Terminate the recording of the ROSbag
  */
  std::string stopRecord(const std::string & robot_ip = "<ROBOT_IP>");

  /**
  * @brief Would insert a message into the bag; currently a no-op
  */
  template<class T>
  void write(
    const std::string & topic, const T & msg,
    const rclcpp::Time & time = helpers::Time::now() )
  {
    // Not stored: bag output is not implemented (see class comment).
    (void)topic;
    (void)msg;
    (void)time;
  }

  void write(
    const std::string & topic,
    const std::vector<geometry_msgs::msg::TransformStamped> & msgtf);

  /**
  * @brief Check if the ROSbag is opened
  */
  bool isStarted();

private:
  std::string prefix_topic_;
  std::mutex process_mutex_;
  std::string bag_name_;
  std::atomic<bool> is_started_;

};  // class GlobalRecorder
}  // namespace recorder
}  // namespace naoqi

#endif
