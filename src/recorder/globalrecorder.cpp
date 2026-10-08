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

#include <naoqi_driver/recorder/globalrecorder.hpp>

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <rmw/rmw.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "helpers/recorder_helpers.hpp"

namespace naoqi
{
namespace recorder
{

namespace
{

/** The name the driver publishes topic under: relative names go in the namespace. */
std::string resolveTopic(std::string topic_namespace, const std::string & topic)
{
  if (!topic.empty() && topic.front() == '/') {
    return topic;
  }
  while (!topic_namespace.empty() && topic_namespace.back() == '/') {
    topic_namespace.pop_back();
  }
  if (!topic_namespace.empty() && topic_namespace.front() != '/') {
    topic_namespace.insert(0, "/");
  }
  return topic_namespace + "/" + topic;
}

/** <working directory>/<prefix>_<date>, numbered if that already exists. */
std::filesystem::path newBagPath(const std::string & prefix)
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char date[32];
  std::strftime(date, sizeof(date), "%Y-%m-%d_%H-%M-%S", &local);
  const std::string name = prefix.empty() ? date : prefix + "_" + date;
  const std::filesystem::path folder = std::filesystem::current_path();
  std::filesystem::path path = folder / name;
  for (int i = 1; std::filesystem::exists(path); ++i) {
    path = folder / (name + "_" + std::to_string(i));
  }
  return path;
}

}  // namespace

GlobalRecorder::GlobalRecorder(const std::string & topic_namespace)
: topic_namespace_(topic_namespace),
  is_started_(false)
{
}

// Out of line: closing the bag needs the complete rosbag2_cpp::Writer.
GlobalRecorder::~GlobalRecorder() = default;

std::string GlobalRecorder::startRecord(const std::string & prefix_bag)
{
  std::lock_guard<std::mutex> lock(process_mutex_);
  if (is_started_) {
    throw std::runtime_error("Already recording to " + bag_name_);
  }
  const std::filesystem::path path = newBagPath(prefix_bag);
  rosbag2_storage::StorageOptions storage;
  storage.uri = path.string();
  storage.storage_id = "sqlite3";
  const std::string format = rmw_get_serialization_format();
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  writer->open(storage, {format, format});

  writer_ = std::move(writer);
  bag_name_ = path.string();
  write_failed_ = false;
  is_started_ = true;
  RCLCPP_INFO_STREAM(helpers::Node::get_logger(), "The bag " << bag_name_ << " is opened");
  return bag_name_;
}

std::string GlobalRecorder::stopRecord(const std::string & robot_ip)
{
  std::unique_ptr<rosbag2_cpp::Writer> writer;
  std::string bag;
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    if (!is_started_) {
      throw std::runtime_error("Cannot stop recording while it has not been started.");
    }
    is_started_ = false;
    writer = std::move(writer_);
    bag.swap(bag_name_);
  }
  writer.reset();  // closes the bag and writes its metadata.yaml
  RCLCPP_INFO_STREAM(helpers::Node::get_logger(), "The bag " << bag << " is closed");

  // Check if we are on a robot
  const char * home = getenv("HOME");
  const std::string cp = home ? home : "";
  if (!(cp.find("nao") == std::string::npos)) {
    RCLCPP_INFO_STREAM(
      helpers::Node::get_logger(),
      "To download this bag on your computer:\n\t$ scp -r nao@" << robot_ip << ":" << bag <<
        " <LOCAL_PATH>");
  }
  return bag;
}

bool GlobalRecorder::isStarted()
{
  return is_started_;
}

void GlobalRecorder::write(
  const std::string & topic,
  const std::vector<geometry_msgs::msg::TransformStamped> & msgtf)
{
  if (msgtf.empty()) {
    return;
  }
  tf2_msgs::msg::TFMessage message;
  message.transforms = msgtf;
  if (helpers::recorder::isZero(msgtf.front().header.stamp)) {
    write(topic, message);
  } else {
    write(topic, message, msgtf.front().header.stamp);
  }
}

void GlobalRecorder::writeSerialized(
  const std::string & topic, const std::string & type,
  std::shared_ptr<rclcpp::SerializedMessage> msg, const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock(process_mutex_);
  if (!writer_) {
    return;
  }
  try {
    writer_->write(msg, resolveTopic(topic_namespace_, topic), type, time);
  } catch (const std::exception & e) {
    if (!write_failed_) {  // once per bag, not for every message
      RCLCPP_ERROR(
        helpers::Node::get_logger(), "Writing to the bag %s failed: %s", bag_name_.c_str(),
        e.what());
    }
    write_failed_ = true;
  }
}

}  // namespace recorder
}  // namespace naoqi
