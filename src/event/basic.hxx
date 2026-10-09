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

#include <functional>
#include <vector>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

namespace naoqi
{

template<typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::EventRegister()
= default;

template<typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::EventRegister(
  const std::string & key,
  const qi::SessionPtr & session)
: key_(key),
  p_memory_(session->service("ALMemory").value()),
  is_started_(false),
  is_publishing_(false),
  is_recording_(false),
  is_dumping_(false)
{
  publisher_ = std::make_shared<Publisher>(key_);
  recorder_ = std::make_shared<Recorder>(key_);
  converter_ = std::make_shared<Converter>(key_, 0, session, key_);

  namespace ph = std::placeholders;
  converter_->registerCallback(
    message_actions::PUBLISH,
    std::bind(&Publisher::publish, publisher_, ph::_1) );
  converter_->registerCallback(
    message_actions::RECORD,
    std::bind(&Recorder::write, recorder_, ph::_1) );
  converter_->registerCallback(
    message_actions::LOG,
    std::bind(&Recorder::bufferize, recorder_, ph::_1) );

  signal_ = p_memory_.call<qi::AnyObject>("subscriber", key_);
}

template<typename Converter, typename Publisher, typename Recorder>
EventRegister<Converter, Publisher, Recorder>::~EventRegister()
= default;

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::resetPublisher(rclcpp::Node * node)
{
  publisher_->reset(node);
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::resetRecorder(
  std::shared_ptr<naoqi::recorder::GlobalRecorder> gr)
{
  recorder_->reset(gr, converter_->frequency());
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::startProcess()
{
  std::lock_guard<std::mutex> start_lock(mutex_);
  if (!is_started_) {
    registerCallback();
    is_started_ = true;
  }
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::stopProcess()
{
  std::lock_guard<std::mutex> stop_lock(mutex_);
  if (is_started_) {
    unregisterCallback();
    is_started_ = false;
  }
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::writeDump(const rclcpp::Time & time)
{
  if (is_started_) {
    recorder_->writeDump(time);
  }
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::setBufferDuration(float duration)
{
  recorder_->setBufferDuration(duration);
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isRecording(bool state)
{
  std::lock_guard<std::mutex> rec_lock(mutex_);
  is_recording_ = state;
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isPublishing(bool state)
{
  std::lock_guard<std::mutex> pub_lock(mutex_);
  is_publishing_ = state;
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::isDumping(bool state)
{
  std::lock_guard<std::mutex> dump_lock(mutex_);
  is_dumping_ = state;
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::registerCallback()
{
  signal_id_ = signal_.connect("signal", [&](qi::AnyValue value) {onEvent();}).value();
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::unregisterCallback()
{
  signal_.disconnect(signal_id_);
}

template<typename Converter, typename Publisher, typename Recorder>
void EventRegister<Converter, Publisher, Recorder>::onEvent()
{
  std::vector<message_actions::MessageAction> actions;
  std::lock_guard<std::mutex> callback_lock(mutex_);
  if (is_started_) {
    // CHECK FOR PUBLISH
    if (is_publishing_ && publisher_->isSubscribed() ) {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if (is_recording_) {
      actions.push_back(message_actions::RECORD);
    }
    if (!is_dumping_) {
      actions.push_back(message_actions::LOG);
    }
    if (!actions.empty()) {
      converter_->callAll(actions);
    }
  }
}

}  // namespace naoqi
