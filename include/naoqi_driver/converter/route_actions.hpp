/*
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

#ifndef NAOQI_DRIVER__CONVERTER__ROUTE_ACTIONS_HPP_
#define NAOQI_DRIVER__CONVERTER__ROUTE_ACTIONS_HPP_

#include <memory>

#include <naoqi_driver/message_actions.h>

namespace naoqi
{
namespace converter
{

/**
 * @brief Routes a converter's PUBLISH, RECORD and LOG actions to publisher->publish(),
 * recorder->write() and recorder->bufferize(). Works for any callback arity (an image
 * and its camera info, joint states and their transforms, ...). The callbacks share
 * ownership of the publisher and recorder, so these live as long as the converter.
 */
template<typename Converter, typename Publisher, typename Recorder>
void routeActions(
  Converter & converter, std::shared_ptr<Publisher> publisher,
  std::shared_ptr<Recorder> recorder)
{
  converter.registerCallback(
    message_actions::PUBLISH, [publisher](auto &&... msg) {publisher->publish(msg ...);});
  converter.registerCallback(
    message_actions::RECORD, [recorder](auto &&... msg) {recorder->write(msg ...);});
  converter.registerCallback(
    message_actions::LOG, [recorder](auto &&... msg) {recorder->bufferize(msg ...);});
}

}  // namespace converter
}  // namespace naoqi

#endif  // NAOQI_DRIVER__CONVERTER__ROUTE_ACTIONS_HPP_
