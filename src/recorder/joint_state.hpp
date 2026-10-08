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

#ifndef JOINT_STATE_RECORDER_HPP
#define JOINT_STATE_RECORDER_HPP

/*
* BOOST includes
*/
#include <naoqi_driver/recorder/recorder.hpp>
#include <memory>
#include <boost/circular_buffer.hpp>

/*
* LOCAL includes
*/
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include "helpers/recorder_helpers.hpp"

/*
* ROS includes
*/
#include <sensor_msgs/msg/joint_state.hpp>

namespace naoqi
{
namespace recorder
{

class JointStateRecorder : public Recorder
{

public:
  JointStateRecorder(const std::string & topic, float buffer_frequency = 0);

  void write(
    const sensor_msgs::msg::JointState & js_msg,
    const std::vector<geometry_msgs::msg::TransformStamped> & tf_transforms);

  void reset(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr, float conv_frequency) override;

  void bufferize(
    const sensor_msgs::msg::JointState & js_msg,
    const std::vector<geometry_msgs::msg::TransformStamped> & tf_transforms);

  void writeDump(const rclcpp::Time & time) override;

  void setBufferDuration(float duration) override;

  std::string topic() const override
  {
    return topic_;
  }

  bool isInitialized() const override
  {
    return is_initialized_;
  }

  void subscribe(bool state) override
  {
    is_subscribed_ = state;
  }

  bool isSubscribed() const override
  {
    return is_subscribed_;
  }

protected:
  std::string topic_;

  boost::circular_buffer<sensor_msgs::msg::JointState> buffer_joint_state_;
  boost::circular_buffer<std::vector<geometry_msgs::msg::TransformStamped>> buffer_tf_;
  size_t buffer_size_;
  float buffer_duration_;

  std::mutex mutex_;

  bool is_initialized_;
  bool is_subscribed_;

  std::shared_ptr<naoqi::recorder::GlobalRecorder> gr_;

  float buffer_frequency_;
  float conv_frequency_;
  int counter_;
  int max_counter_;

}; // class

}  // namespace recorder
}  // namespace naoqi

#endif
