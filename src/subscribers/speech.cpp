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
#include "speech.hpp"
#include "helpers/async_helpers.hpp"
#include <naoqi_driver/ros_helpers.hpp>

namespace naoqi
{
namespace subscriber
{

SpeechSubscriber::SpeechSubscriber(
  const std::string & name, const std::string & speech_topic,
  const qi::SessionPtr & session)
: speech_topic_(speech_topic),
  BaseSubscriber(name, speech_topic, session),
  p_tts_(session->service("ALTextToSpeech").value())
{}

void SpeechSubscriber::reset(rclcpp::Node * node)
{
  sub_speech_ = node->create_subscription<std_msgs::msg::String>(
    speech_topic_,
    10,
    std::bind(&SpeechSubscriber::speechCallback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void SpeechSubscriber::speechCallback(const std_msgs::msg::String::SharedPtr string_msg)
{
  helpers::warnOnAsyncError(
    p_tts_.async<void>("say", string_msg->data), naoqi::helpers::Node::get_logger(),
    "ALTextToSpeech.say");
}

}  // namespace subscriber
}  // namespace naoqi
