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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

/*
* ROS includes
*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

namespace rosbag2_cpp
{
class Writer;
}  // namespace rosbag2_cpp

namespace naoqi
{
namespace recorder
{

/**
* @brief Writes the messages of all recorders into one rosbag2 bag at a time.
* All methods are thread-safe: converters, events and services write concurrently.
*/
class GlobalRecorder
{
public:
  /**
  * @param topic_namespace namespace of relative topic names, as for the driver's
  * publishers ("/" or "" records "imu/torso" as "/imu/torso")
  */
  explicit GlobalRecorder(const std::string & topic_namespace);
  ~GlobalRecorder();

  /**
  * @brief Opens a new bag, a folder named <prefix_bag>_<date> in the working directory
  * @return the bag folder
  * @throw std::runtime_error if a bag is already open or it cannot be created
  */
  std::string startRecord(const std::string & prefix_bag = "naoqi_driver");

  /**
  * @brief Closes the bag
  * @return the bag folder
  * @throw std::runtime_error if no bag is open
  */
  std::string stopRecord(const std::string & robot_ip = "<ROBOT_IP>");

  /**
  * @brief Adds a message to the open bag (no-op when none is open)
  */
  template<class T>
  void write(
    const std::string & topic, const T & msg,
    const rclcpp::Time & time = helpers::Time::now() )
  {
    if (!is_started_) {
      return;
    }
    auto serialized = std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<T>().serialize_message(&msg, serialized.get());
    writeSerialized(topic, rosidl_generator_traits::name<T>(), serialized, time);
  }

  /**
  * @brief Adds transforms to the open bag as one tf2_msgs/TFMessage
  */
  void write(
    const std::string & topic,
    const std::vector<geometry_msgs::msg::TransformStamped> & msgtf);

  /**
  * @brief Check if the ROSbag is opened
  */
  bool isStarted();

private:
  void writeSerialized(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> msg, const rclcpp::Time & time);

  std::string topic_namespace_;
  std::mutex process_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string bag_name_;
  bool write_failed_ = false;
  std::atomic<bool> is_started_;

};  // class GlobalRecorder
}  // namespace recorder
}  // namespace naoqi

#endif
