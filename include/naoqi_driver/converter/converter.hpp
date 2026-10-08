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

#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <string>
#include <vector>

#include <naoqi_driver/message_actions.h>

namespace naoqi
{
namespace converter
{

/**
 * @brief Turns NAOqi data into ROS messages on a schedule. Implemented through
 * converter::BaseConverter. Each call to callAll() fetches the data once and runs the
 * requested actions (publish, record, buffer) on it.
 */
class Converter
{
public:
  virtual ~Converter() = default;

  /** Descriptive name; also the key of its publisher and recorder. */
  virtual std::string name() const = 0;

  /** Hz; 0 means "run once". */
  virtual float frequency() const = 0;

  virtual void reset() = 0;

  virtual void callAll(const std::vector<message_actions::MessageAction> & actions) = 0;
};

}  // namespace converter
}  // namespace naoqi

#endif
