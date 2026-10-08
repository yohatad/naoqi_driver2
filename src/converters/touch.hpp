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

#ifndef TOUCH_EVENT_CONVERTER_HPP
#define TOUCH_EVENT_CONVERTER_HPP

/*
* LOCAL includes
*/
#include <functional>
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <naoqi_bridge_msgs/msg/bumper.hpp>
#include <naoqi_bridge_msgs/msg/hand_touch.hpp>
#include <naoqi_bridge_msgs/msg/head_touch.hpp>

/*
* ALDEBARAN includes
*/
#include <qi/anymodule.hpp>

namespace naoqi
{

namespace converter
{

template<class T>
class TouchEventConverter : public BaseConverter<TouchEventConverter<T>>
{

  using Callback_t = std::function<void (T &)>;

public:
  TouchEventConverter(
    const std::string & name, float frequency,
    const qi::SessionPtr & session);

  ~TouchEventConverter() override;

  void reset() override;

  // Event-driven: messages arrive through callAll(actions, msg), never on a schedule.
  void callAll(const std::vector<message_actions::MessageAction> &) override {}

  void registerCallback(const message_actions::MessageAction action, Callback_t cb);

  void callAll(const std::vector<message_actions::MessageAction> & actions, T & msg);

private:
  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;
  T msg_;
};

}  // namespace converter

}  // namespace naoqi

#endif // TOUCH_CONVERTER_HPP
