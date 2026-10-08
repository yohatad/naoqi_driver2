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
#include <memory>
#include "sonar.hpp"

namespace naoqi
{
namespace recorder
{

SonarRecorder::SonarRecorder(const std::vector<std::string> & topics, float buffer_frequency)
: topics_(topics),
  buffer_duration_(helpers::recorder::bufferDefaultDuration),
  buffer_frequency_(buffer_frequency),
  counter_(1)
{}

void SonarRecorder::write(const std::vector<sensor_msgs::msg::Range> & sonar_msgs)
{
  if (topics_.size() != sonar_msgs.size() ) {
    RCLCPP_ERROR_STREAM(
      naoqi::helpers::Node::get_logger(),
      "Incorrect number of sonar range messages in sonar recorder. " << sonar_msgs.size() << "/" <<
        topics_.size());
    return;
  }

  for (size_t i = 0; i < sonar_msgs.size(); ++i) {
    if (!helpers::recorder::isZero(sonar_msgs[i].header.stamp)) {
      gr_->write(topics_[i], sonar_msgs[i], sonar_msgs[i].header.stamp);
    } else {
      gr_->write(topics_[i], sonar_msgs[i]);
    }
  }
}

void SonarRecorder::writeDump(const rclcpp::Time & time)
{
  std::lock_guard<std::mutex> lock_write_buffer(mutex_);
  for (auto & ranges : buffer_) {
    write(ranges);
  }
}

void SonarRecorder::reset(std::shared_ptr<GlobalRecorder> gr, float conv_frequency)
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
  buffer_.set_capacity(buffer_size_);
  is_initialized_ = true;
}

void SonarRecorder::bufferize(const std::vector<sensor_msgs::msg::Range> & sonar_msgs)
{
  std::lock_guard<std::mutex> lock_bufferize(mutex_);
  if (counter_ < max_counter_) {
    counter_++;
  } else {
    counter_ = 1;
    buffer_.push_back(sonar_msgs);
  }
}

void SonarRecorder::setBufferDuration(float duration)
{
  std::lock_guard<std::mutex> lock_bufferize(mutex_);
  buffer_size_ = static_cast<size_t>(duration * (conv_frequency_ / max_counter_));
  buffer_duration_ = duration;
  buffer_.set_capacity(buffer_size_);
}

}  // namespace recorder
}  // namespace naoqi
