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

#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "naoqi_driver/recorder/globalrecorder.hpp"

namespace naoqi
{
namespace recorder
{

/** @brief Records a converter's messages and keeps a buffer of the last ones for dumps. */
class Recorder
{
public:
  virtual ~Recorder() = default;

  /** Topic recorded, relative to the bag's prefix. */
  virtual std::string topic() const = 0;

  virtual bool isInitialized() const = 0;

  /** Includes this topic in, or excludes it from, the next recordings. */
  virtual void subscribe(bool state) = 0;

  virtual bool isSubscribed() const = 0;

  /** Attaches the shared bag state and sizes the buffer for the converter frequency. */
  virtual void reset(std::shared_ptr<GlobalRecorder> gr, float conv_frequency) = 0;

  /** Writes the buffered messages to the bag. */
  virtual void writeDump(const rclcpp::Time & time) = 0;

  /** Seconds of messages kept for writeDump(). */
  virtual void setBufferDuration(float duration) = 0;
};

}  // namespace recorder
}  // namespace naoqi

#endif
