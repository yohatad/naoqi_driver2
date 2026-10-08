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
#include <memory>
#include "joint_state.hpp"

namespace naoqi
{
namespace recorder
{

JointStateRecorder::JointStateRecorder(const std::string & topic, float buffer_frequency)
: topic_(topic),
  buffer_duration_(helpers::recorder::bufferDefaultDuration),
  is_initialized_(false),
  is_subscribed_(false),
  buffer_frequency_(buffer_frequency),
  counter_(1)
{}

void JointStateRecorder::write(
  const sensor_msgs::msg::JointState & js_msg,
  const std::vector<geometry_msgs::msg::TransformStamped> & tf_transforms)
{
  if (!helpers::recorder::isZero(js_msg.header.stamp)) {
    gr_->write(topic_, js_msg, js_msg.header.stamp);
  } else {
    gr_->write(topic_, js_msg);
  }
  gr_->write("/tf", tf_transforms);
}

void JointStateRecorder::writeDump(const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock_write_buffer(mutex_);
  for (auto & transforms : buffer_tf_) {
    gr_->write("/tf", transforms);
  }
  for (auto & joint_state : buffer_joint_state_) {
    if (!helpers::recorder::isZero(joint_state.header.stamp)) {
      gr_->write(topic_, joint_state, joint_state.header.stamp);
    } else {
      gr_->write(topic_, joint_state);
    }
  }
}

void JointStateRecorder::reset(std::shared_ptr<GlobalRecorder> gr, float conv_frequency)
{
  gr_ = gr;
  conv_frequency_ = conv_frequency;
  if (buffer_frequency_ != 0) {
    max_counter_ = static_cast<int>(conv_frequency / buffer_frequency_);
    buffer_size_ = static_cast<size_t>(buffer_duration_ * (conv_frequency / max_counter_));
  } else {
    max_counter_ = 1;
    buffer_size_ = static_cast<size_t>(buffer_duration_ * conv_frequency);
  }
  buffer_joint_state_.set_capacity(buffer_size_);
  buffer_tf_.set_capacity(buffer_size_);
  is_initialized_ = true;
}

void JointStateRecorder::bufferize(
  const sensor_msgs::msg::JointState & js_msg,
  const std::vector<geometry_msgs::msg::TransformStamped> & tf_transforms)
{
  std::lock_guard<std::mutex> lock_bufferize(mutex_);
  if (counter_ < max_counter_) {
    counter_++;
  } else {
    counter_ = 1;
    buffer_joint_state_.push_back(js_msg);
    buffer_tf_.push_back(tf_transforms);
  }
}

void JointStateRecorder::setBufferDuration(float duration)
{
  std::lock_guard<std::mutex> lock_bufferize(mutex_);
  buffer_size_ = static_cast<size_t>(duration * (conv_frequency_ / max_counter_));
  buffer_duration_ = duration;
  buffer_joint_state_.set_capacity(buffer_size_);
  buffer_tf_.set_capacity(buffer_size_);
}

}  // namespace recorder
}  // namespace naoqi
