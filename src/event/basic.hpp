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

#ifndef EVENT_REGISTER_HPP
#define EVENT_REGISTER_HPP

#include <string>

#include <memory>
#include <atomic>
#include <mutex>

#include <qi/session.hpp>

#include <rclcpp/rclcpp.hpp>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
template<typename Converter, typename Publisher, typename Recorder>
class EventRegister
{

public:
  /**
  * @brief Constructor for recorder interface
  */
  EventRegister();
  EventRegister(const std::string & key, const qi::SessionPtr & session);
  ~EventRegister();

  void resetPublisher(rclcpp::Node * node);
  void resetRecorder(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr);

  void startProcess();
  void stopProcess();

  void writeDump(const rclcpp::Time & time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  std::shared_ptr<Converter> converter_;
  std::shared_ptr<Publisher> publisher_;
  std::shared_ptr<Recorder> recorder_;

  qi::AnyObject p_memory_;
  qi::AnyObject signal_;
  qi::SignalLink signal_id_;
  std::string key_;

  std::mutex mutex_;

  std::atomic<bool> is_started_;
  bool is_publishing_;
  bool is_recording_;
  bool is_dumping_;

}; // class globalrecorder
}  // namespace naoqi
#include "basic.hxx"

#endif
