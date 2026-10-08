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
#include <naoqi_driver/ros_helpers.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include "helpers/recorder_helpers.hpp"

/*
* STANDARD includes
*/
#include <ctime>
#include <sstream>

/*
* BOOST includes
*/
#include <filesystem>
#include <memory>

/*
* ALDEBARAN includes
*/
#include <qi/log.hpp>

qiLogCategory("ros.Recorder");

namespace naoqi
{
namespace recorder
{

GlobalRecorder::GlobalRecorder(const std::string & prefix_topic)
: process_mutex_()
  , bag_name_("")
  , is_started_(false)
{
  if (!prefix_topic.empty()) {
    prefix_topic_ = "/" + prefix_topic + "/";
  } else {
    prefix_topic_ = "/";
  }
}

void GlobalRecorder::startRecord(const std::string & prefix_bag)
{
  std::lock_guard<std::mutex> startLock(process_mutex_);
  if (!is_started_) {
    try {
      // Get current path
      std::filesystem::path cur_path(std::filesystem::current_path() );

      // Get time
      time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];
      std::time(&rawtime);
      timeinfo = std::localtime(&rawtime);
      std::strftime(buffer, 80, "%d-%m-%Y_%I:%M:%S", timeinfo);

      if (!prefix_bag.empty()) {
        bag_name_ = cur_path.string() + "/" + prefix_bag + "_" + buffer;
      } else {
        bag_name_ = cur_path.string() + "/" + buffer;
      }
      bag_name_.append(".bag");
      is_started_ = true;
      RCLCPP_INFO_STREAM(
        naoqi::helpers::Node::get_logger(),
        "The bag " << bag_name_ << " is opened");
    } catch (const std::exception & e) {
      throw std::runtime_error(e.what());
    }
  } else {
    qiLogError() << "Cannot start a record. The module is already recording.";
  }
}

std::string GlobalRecorder::stopRecord(const std::string & robot_ip)
{
  std::lock_guard<std::mutex> stopLock(process_mutex_);
  if (is_started_) {
    is_started_ = false;

    std::stringstream message;
    message << bag_name_;
    RCLCPP_INFO_STREAM(naoqi::helpers::Node::get_logger(), "The bag " << bag_name_ << " is closed");

    // Check if we are on a robot
    const char * home = getenv("HOME");
    const std::string cp = home ? home : "";
    if (!(cp.find("nao") == std::string::npos)) {
      RCLCPP_INFO_STREAM(
        naoqi::helpers::Node::get_logger(),
        "To download this bag on your computer:\n\t$ scp nao@" << robot_ip << ":" << bag_name_ <<
          " <LOCAL_PATH>");
    }

    bag_name_.clear();
    return message.str();
  } else {
    qiLogError() << "Cannot stop recording while it has not been started.";
    return "Cannot stop recording while it has not been started.";
  }
}

bool GlobalRecorder::isStarted()
{
  return is_started_;
}

void GlobalRecorder::write(
  const std::string & topic,
  const std::vector<geometry_msgs::msg::TransformStamped> & msgtf)
{
  // Not stored: bag output is not implemented (see class comment).
  (void)topic;
  (void)msgtf;
}

}  // namespace recorder
}  // namespace naoqi
