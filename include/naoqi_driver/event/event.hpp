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

#ifndef EVENT_HPP
#define EVENT_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>

namespace naoqi
{
namespace event
{

/**
 * @brief Publishes and records data that NAOqi pushes as it happens (audio buffers,
 * touch and memory events), as opposed to converters, which poll on a schedule.
 */
class Event
{
public:
  virtual ~Event() = default;

  /** Creates (or re-creates) the ROS publisher on the given node. */
  virtual void resetPublisher(rclcpp::Node * node) = 0;

  /** Attaches the shared bag state. */
  virtual void resetRecorder(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr) = 0;

  /** Starts / stops listening to NAOqi. */
  virtual void startProcess() = 0;
  virtual void stopProcess() = 0;

  /** Writes the buffered messages to the bag. */
  virtual void writeDump(const rclcpp::Time & time) = 0;

  /** Seconds of messages kept for writeDump(). */
  virtual void setBufferDuration(float duration) = 0;

  /** Which actions run on each new message. */
  virtual void isRecording(bool state) = 0;
  virtual void isPublishing(bool state) = 0;
  virtual void isDumping(bool state) = 0;
};

}  // namespace event
}  // namespace naoqi

#endif
