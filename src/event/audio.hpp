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

#ifndef AUDIO_EVENT_REGISTER_HPP
#define AUDIO_EVENT_REGISTER_HPP

#include <memory>
#include <string>

#include <atomic>
#include <mutex>

#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <rclcpp/rclcpp.hpp>
#include <naoqi_bridge_msgs/msg/audio_buffer.h>
#include <naoqi_driver/ros_helpers.hpp>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "converters/audio.hpp"
// Publisher
#include "publishers/basic.hpp"
// Recorder
#include "recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief Streams the robot microphones as naoqi_bridge_msgs/AudioBuffer.
* @note Registered as a qi service, so ALAudioDevice pushes buffers into processRemote().
*/
// Stays on boost::enable_shared_from_this: startProcess() registers this object as a
// qi service, and qi::Object can only be built from a boost::shared_ptr.
class AudioEventRegister : public boost::enable_shared_from_this<AudioEventRegister>
{

public:
  /**
  * @brief Constructor for recorder interface
  */
  AudioEventRegister(
    const std::string & name, float frequency,
    const qi::SessionPtr & session);
  ~AudioEventRegister();

  void resetPublisher(rclcpp::Node * node);
  void resetRecorder(std::shared_ptr<naoqi::recorder::GlobalRecorder> gr);

  void startProcess();
  void stopProcess();

  void writeDump(const rclcpp::Time & time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void processRemote(
    int nbOfChannels, int samplesByChannel, qi::AnyValue altimestamp,
    qi::AnyValue buffer);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  qi::SessionPtr session_;
  publisher::BasicPublisher<naoqi_bridge_msgs::msg::AudioBuffer> publisher_;
  recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::AudioBuffer> recorder_;
  converter::AudioEventConverter converter_;
  qi::AnyObject p_audio_;
  qi::FutureSync<qi::AnyObject> p_audio_extractor_request_;
  std::vector<uint8_t> channel_map_;
  unsigned int service_id_;

  std::mutex subscription_mutex_;
  std::mutex processing_mutex_;

  std::atomic<bool> is_started_;
  bool is_publishing_;
  bool is_recording_;
  bool is_dumping_;

}; // class

QI_REGISTER_OBJECT(AudioEventRegister, processRemote)

}  // namespace naoqi

#endif
