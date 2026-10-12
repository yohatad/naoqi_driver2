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

#include <naoqi_driver/ros_helpers.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <vector>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

#include "touch.hpp"
#include "tools/from_any_value.hpp"

namespace naoqi
{

namespace
{

constexpr auto kPollPeriod = std::chrono::milliseconds(50);

/**
 * The ALMemory sensor values that can stand behind a touch event, on Pepper and on NAO;
 * the ones the robot does not have are dropped at start-up.
 */
std::vector<std::string> sensorKeyCandidates(const std::string & event)
{
  static const std::string prefix = "Device/SubDeviceList/";
  static const std::string suffix = "/Sensor/Value";
  static const std::map<std::string, std::vector<std::string>> devices = {
    {"RightBumperPressed",
      {"Platform/FrontRight/Bumper", "RFoot/Bumper/Left", "RFoot/Bumper/Right"}},
    {"LeftBumperPressed", {"Platform/FrontLeft/Bumper", "LFoot/Bumper/Left", "LFoot/Bumper/Right"}},
    {"BackBumperPressed", {"Platform/Back/Bumper"}},
    {"HandRightBackTouched", {"RHand/Touch/Back"}},
    {"HandRightLeftTouched", {"RHand/Touch/Left"}},
    {"HandRightRightTouched", {"RHand/Touch/Right"}},
    {"HandLeftBackTouched", {"LHand/Touch/Back"}},
    {"HandLeftLeftTouched", {"LHand/Touch/Left"}},
    {"HandLeftRightTouched", {"LHand/Touch/Right"}},
    {"FrontTactilTouched", {"Head/Touch/Front"}},
    {"MiddleTactilTouched", {"Head/Touch/Middle"}},
    {"RearTactilTouched", {"Head/Touch/Rear"}}};
  std::vector<std::string> keys;
  const auto it = devices.find(event);
  if (it != devices.end()) {
    for (const auto & device : it->second) {
      keys.push_back(prefix + device + suffix);
    }
  }
  return keys;
}

}  // namespace

template<class T>
TouchEventRegister<T>::TouchEventRegister()
= default;

template<class T>
TouchEventRegister<T>::TouchEventRegister(
  const std::string & name,
  const std::vector<std::string> keys,
  float frequency, const qi::SessionPtr & session)
: p_memory_(session->service("ALMemory").value()),
  is_started_(false),
  is_publishing_(false),
  is_recording_(false),
  is_dumping_(false)
{
  publisher_ = std::make_shared<publisher::BasicPublisher<T>>(name);
  converter_ = std::make_shared<converter::TouchEventConverter<T>>(name, frequency, session);

  converter_->registerCallback(
    message_actions::PUBLISH,
    std::bind(&publisher::BasicPublisher<T>::publish, publisher_, std::placeholders::_1) );
  // Touch events are only published; they are never recorded or buffered.

  keys_ = keys;
  name_ = name;

  // Keep the sensor values this robot has; an event with none of them is never reported.
  const auto available = p_memory_.call<std::vector<std::string>>("getDataListName");
  for (const auto & key : keys_) {
    std::vector<std::string> sensors;
    for (const auto & candidate : sensorKeyCandidates(key)) {
      if (std::find(available.begin(), available.end(), candidate) != available.end()) {
        sensors.push_back(candidate);
        sensor_key_list_.push_back(candidate);
      }
    }
    if (sensors.empty()) {
      RCLCPP_WARN_STREAM(
        naoqi::helpers::Node::get_logger(), name_ << ": no sensor for " << key << ", ignoring it");
    }
    sensor_keys_.push_back(sensors);
  }
}

template<class T>
TouchEventRegister<T>::~TouchEventRegister()
{
  stopProcess();
}

template<class T>
void TouchEventRegister<T>::resetPublisher(rclcpp::Node * node)
{
  publisher_->reset(node);
}

template<class T>
void TouchEventRegister<T>::resetRecorder(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr)
{
  (void)gr;  // not recorded
}

template<class T>
void TouchEventRegister<T>::startProcess()
{
  std::lock_guard<std::mutex> start_lock(mutex_);
  if (!is_started_) {
    {
      std::lock_guard<std::mutex> poll_lock(poll_mutex_);
      stop_polling_ = false;
    }
    poll_thread_ = std::thread(&TouchEventRegister<T>::pollLoop, this);
    is_started_ = true;
  }
}

template<class T>
void TouchEventRegister<T>::stopProcess()
{
  {
    std::lock_guard<std::mutex> stop_lock(mutex_);
    if (!is_started_) {
      return;
    }
    is_started_ = false;
    std::lock_guard<std::mutex> poll_lock(poll_mutex_);
    stop_polling_ = true;
  }
  poll_cv_.notify_all();
  // Joined without mutex_ held: the poll thread takes it to report a change.
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

template<class T>
std::vector<bool> TouchEventRegister<T>::readStates()
{
  std::vector<float> values;
  if (!sensor_key_list_.empty()) {
    qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", sensor_key_list_);
    tools::fromAnyValueToFloatVector(anyvalues, values);
  }
  std::vector<bool> states(keys_.size(), false);
  size_t value = 0;
  for (size_t i = 0; i < keys_.size(); ++i) {
    for (size_t j = 0; j < sensor_keys_[i].size(); ++j, ++value) {
      if (value < values.size() && values[value] > 0.5f) {
        states[i] = true;
      }
    }
  }
  return states;
}

template<class T>
void TouchEventRegister<T>::pollLoop()
{
  // The first read sets the reference: only later presses and releases are reported.
  std::vector<bool> last_states;
  bool failing = false;
  std::unique_lock<std::mutex> poll_lock(poll_mutex_);
  while (!stop_polling_) {
    poll_lock.unlock();
    try {
      const std::vector<bool> states = readStates();
      if (!last_states.empty()) {
        for (size_t i = 0; i < states.size(); ++i) {
          if (states[i] != last_states[i]) {
            touchCallback(keys_[i], states[i]);
          }
        }
      }
      last_states = states;
      failing = false;
    } catch (const std::exception & e) {
      if (!failing) {  // once per run of failures, not every 50 ms
        RCLCPP_ERROR_STREAM(
          naoqi::helpers::Node::get_logger(), "Could not read " << name_ << ": " << e.what());
      }
      failing = true;
    }
    poll_lock.lock();
    poll_cv_.wait_for(poll_lock, kPollPeriod, [this] {return stop_polling_;});
  }
}

template<class T>
void TouchEventRegister<T>::writeDump(const rclcpp::Time & time)
{
  (void)time;  // not recorded
}

template<class T>
void TouchEventRegister<T>::setBufferDuration(float duration)
{
  (void)duration;  // not recorded
}

template<class T>
void TouchEventRegister<T>::isRecording(bool state)
{
  std::lock_guard<std::mutex> rec_lock(mutex_);
  is_recording_ = state;
}

template<class T>
void TouchEventRegister<T>::isPublishing(bool state)
{
  std::lock_guard<std::mutex> pub_lock(mutex_);
  is_publishing_ = state;
}

template<class T>
void TouchEventRegister<T>::isDumping(bool state)
{
  std::lock_guard<std::mutex> dump_lock(mutex_);
  is_dumping_ = state;
}

template<class T>
void TouchEventRegister<T>::touchCallback(const std::string & key, bool state)
{
  T msg = T();

  touchCallbackMessage(key, state, msg);

  std::vector<message_actions::MessageAction> actions;
  std::lock_guard<std::mutex> callback_lock(mutex_);
  if (is_started_) {
    // CHECK FOR PUBLISH
    if (is_publishing_ && publisher_->isSubscribed() ) {
      actions.push_back(message_actions::PUBLISH);
    }
    if (!actions.empty()) {
      converter_->callAll(actions, msg);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(
  const std::string & key, bool & state,
  naoqi_bridge_msgs::msg::Bumper & msg)
{
  int i = 0;
  for (std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i) {
    if (key == it->c_str() ) {
      msg.bumper = i;
      msg.state =
        state ? (naoqi_bridge_msgs::msg::Bumper::STATE_PRESSED) : (naoqi_bridge_msgs::msg::Bumper
        ::STATE_RELEASED);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(
  const std::string & key, bool & state,
  naoqi_bridge_msgs::msg::HandTouch & msg)
{
  int i = 0;
  for (std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i) {
    if (key == it->c_str() ) {
      msg.hand = i;
      msg.state =
        state ? (naoqi_bridge_msgs::msg::HandTouch::STATE_PRESSED) : (naoqi_bridge_msgs::msg::
        HandTouch::STATE_RELEASED);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(
  const std::string & key, bool & state,
  naoqi_bridge_msgs::msg::HeadTouch & msg)
{
  int i = 0;
  for (std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i) {
    if (key == it->c_str() ) {
      msg.button = i;
      msg.state =
        state ? (naoqi_bridge_msgs::msg::HeadTouch::STATE_PRESSED) : (naoqi_bridge_msgs::msg::
        HeadTouch::STATE_RELEASED);
    }
  }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class TouchEventRegister<naoqi_bridge_msgs::msg::Bumper>;
template class TouchEventRegister<naoqi_bridge_msgs::msg::HandTouch>;
template class TouchEventRegister<naoqi_bridge_msgs::msg::HeadTouch>;

}  // namespace naoqi
