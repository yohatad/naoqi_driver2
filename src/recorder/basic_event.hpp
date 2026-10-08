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

#ifndef BASIC_EVENT_RECORDER_HPP
#define BASIC_EVENT_RECORDER_HPP

/*
* LOCAL includes
*/
#include <memory>
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/ros_helpers.hpp>
#include "helpers/recorder_helpers.hpp"
#include <rclcpp/duration.hpp>

/*
* STANDARD includes
*/
#include <string>

namespace naoqi
{
namespace recorder
{

template<class T>
class BasicEventRecorder
{

public:
  BasicEventRecorder(const std::string & topic)
  : topic_(topic),
    buffer_duration_(helpers::recorder::bufferDefaultDuration),
    is_initialized_(false),
    is_subscribed_(false)
  {}

  virtual ~BasicEventRecorder() = default;

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline void subscribe(bool state)
  {
    is_subscribed_ = state;
  }

  inline bool isSubscribed() const
  {
    return is_subscribed_;
  }

  virtual void write(const T & msg)
  {
    if (!helpers::recorder::isZero(msg.header.stamp)) {
      gr_->write(topic_, msg, msg.header.stamp);
    } else {
      gr_->write(topic_, msg);
    }
  }

  virtual void writeDump(const rclcpp::Time & time)
  {
    std::lock_guard<std::mutex> lock_write_buffer(mutex_);
    removeOlderThan(time);
    for (auto & msg : buffer_) {
      if (!helpers::recorder::isZero(msg.header.stamp)) {
        gr_->write(topic_, msg, msg.header.stamp);
      } else {
        gr_->write(topic_, msg);
      }
    }
  }

  virtual void bufferize(const T & msg)
  {
    std::lock_guard<std::mutex> lock_bufferize(mutex_);
    typename std::list<T>::iterator it;
    removeOld();
    buffer_.push_back(msg);

  }

  virtual void reset(std::shared_ptr<GlobalRecorder> gr, float conv_frequency)
  {
    gr_ = gr;
    is_initialized_ = true;
  }

  virtual void setBufferDuration(float duration)
  {
    std::lock_guard<std::mutex> lock_bufferize(mutex_);
    buffer_duration_ = duration;
  }

protected:
  bool isTooOld(const T & msg)
  {
    rclcpp::Duration d(helpers::Time::now() - msg.header.stamp);
    return static_cast<float>(d.seconds()) > buffer_duration_;
  }

  bool isOlderThan(const T & msg, const rclcpp::Time & time)
  {
    rclcpp::Duration d(time - msg.header.stamp);
    return static_cast<float>(d.seconds()) > buffer_duration_;
  }

  void removeOld()
  {
    while (!buffer_.empty() && isTooOld(buffer_.front())) {
      buffer_.pop_front();
    }
  }

  void removeOlderThan(const rclcpp::Time & time)
  {
    while (!buffer_.empty() && isOlderThan(buffer_.front(), time)) {
      buffer_.pop_front();
    }
  }

  std::string topic_;

  std::list<T> buffer_;
  float buffer_duration_;
  std::mutex mutex_;

  bool is_initialized_;
  bool is_subscribed_;

  std::shared_ptr<naoqi::recorder::GlobalRecorder> gr_;
}; // class

}  // namespace recorder
}  // namespace naoqi

#endif
