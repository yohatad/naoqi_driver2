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
#include <functional>
#include <iostream>
#include <vector>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

#include "touch.hpp"

namespace naoqi
{

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

  keys_.resize(keys.size());
  size_t i = 0;
  for (std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i) {
    keys_[i] = *it;
  }

  name_ = name;
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
    if (subscriptions_.empty()) {
      for (const auto & key : keys_) {
        auto subscriber = p_memory_.call<qi::AnyObject>("subscriber", key);
        qi::SignalLink subscription = subscriber.connect(
          "signal", [ = ](const qi::AnyValue & v) {
            touchCallback(key, v);
          }).value();
        subscriptions_.emplace_back(std::move(subscriber), std::move(subscription));
      }
    }
    is_started_ = true;
  }
}

template<class T>
void TouchEventRegister<T>::stopProcess()
{
  std::lock_guard<std::mutex> stop_lock(mutex_);
  if (is_started_) {
    if (!subscriptions_.empty()) {
      for (const auto & subscription : subscriptions_) {
        try {
          subscription.subscriber.disconnect(subscription.link).value();
        } catch (const std::exception & e) {
          RCLCPP_ERROR_STREAM(
            naoqi::helpers::Node::get_logger(),
            "Error attempting to clean-up ALMemory subscription: " << e.what());
        }
      }
      subscriptions_.clear();
    }
    is_started_ = false;
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
void TouchEventRegister<T>::registerCallback()
{
}

template<class T>
void TouchEventRegister<T>::unregisterCallback()
{
}

template<class T>
void TouchEventRegister<T>::touchCallback(const std::string & key, const qi::AnyValue & value)
{
  T msg = T();
  bool state = value.toFloat() > 0.5f;

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
