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

#ifndef TOUCH_EVENT_REGISTER_HPP
#define TOUCH_EVENT_REGISTER_HPP

#include <naoqi_driver/event/event.hpp>
#include <string>
#include <tuple>

#include <memory>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include <qi/session.hpp>

#include <rclcpp/rclcpp.hpp>
#include <naoqi_bridge_msgs/msg/bumper.hpp>
#include <naoqi_bridge_msgs/msg/hand_touch.hpp>
#include <naoqi_bridge_msgs/msg/head_touch.hpp>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "converters/touch.hpp"
// Publisher
#include "publishers/basic.hpp"
// Recorder
#include "recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
template<class T>
class TouchEventRegister
  : public std::enable_shared_from_this<TouchEventRegister<T>>, public event::Event
{

public:
  /**
  * @brief Constructor for recorder interface
  */
  TouchEventRegister();
  TouchEventRegister(
    const std::string & name, const std::vector<std::string> keys,
    float frequency, const qi::SessionPtr & session);
  ~TouchEventRegister() override;

  void resetPublisher(rclcpp::Node * node) override;
  void resetRecorder(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr) override;

  void startProcess() override;
  void stopProcess() override;

  void writeDump(const rclcpp::Time & time) override;
  void setBufferDuration(float duration) override;

  void isRecording(bool state) override;
  void isPublishing(bool state) override;
  void isDumping(bool state) override;

private:
  void touchCallback(const std::string & key, bool state);
  void touchCallbackMessage(
    const std::string & key, bool & state,
    naoqi_bridge_msgs::msg::Bumper & msg);
  void touchCallbackMessage(
    const std::string & key, bool & state,
    naoqi_bridge_msgs::msg::HandTouch & msg);
  void touchCallbackMessage(
    const std::string & key, bool & state,
    naoqi_bridge_msgs::msg::HeadTouch & msg);

  /**
   * Reads the touch sensors' values from ALMemory and reports each change of state.
   * ALMemory.subscriber() would push events instead, but the object it returns cannot be
   * decoded by this libqi when talking to NAOqi 2.5 ("Invalid signature").
   */
  void pollLoop();
  /** One getListData round trip: whether each of keys_ is pressed. Throws on NAOqi errors. */
  std::vector<bool> readStates();

private:
  std::shared_ptr<converter::TouchEventConverter<T>> converter_;
  std::shared_ptr<publisher::BasicPublisher<T>> publisher_;

  qi::SessionPtr session_;
  qi::AnyObject p_memory_;

  // sensor_keys_[i] are the ALMemory sensor values behind the event keys_[i]; it is pressed
  // when any of them is. sensor_key_list_ is all of them, in that order, for getListData.
  std::vector<std::vector<std::string>> sensor_keys_;
  std::vector<std::string> sensor_key_list_;

  std::thread poll_thread_;
  std::mutex poll_mutex_;
  std::condition_variable poll_cv_;
  bool stop_polling_ = false;

  std::string name_;

  std::mutex mutex_;

  std::atomic<bool> is_started_;
  bool is_publishing_;
  bool is_recording_;
  bool is_dumping_;

protected:
  std::vector<std::string> keys_;
}; // class

class BumperEventRegister : public TouchEventRegister<naoqi_bridge_msgs::msg::Bumper>
{
public:
  BumperEventRegister(
    const std::string & name, const std::vector<std::string> keys,
    float frequency, const qi::SessionPtr & session)
  : TouchEventRegister<naoqi_bridge_msgs::msg::Bumper>(name, keys, frequency, session) {}
};

class HeadTouchEventRegister : public TouchEventRegister<naoqi_bridge_msgs::msg::HeadTouch>
{
public:
  HeadTouchEventRegister(
    const std::string & name, const std::vector<std::string> keys,
    float frequency, const qi::SessionPtr & session)
  : TouchEventRegister<naoqi_bridge_msgs::msg::HeadTouch>(name, keys, frequency, session) {}
};

class HandTouchEventRegister : public TouchEventRegister<naoqi_bridge_msgs::msg::HandTouch>
{
public:
  HandTouchEventRegister(
    const std::string & name, const std::vector<std::string> keys,
    float frequency, const qi::SessionPtr & session)
  : TouchEventRegister<naoqi_bridge_msgs::msg::HandTouch>(name, keys, frequency, session) {}
};

}  // namespace naoqi

#endif
