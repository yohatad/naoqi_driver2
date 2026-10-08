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


#ifndef LED_VALIDATION_HPP
#define LED_VALIDATION_HPP

#include <cmath>
#include <string>

#include <naoqi_bridge_msgs/action/run_led.hpp>

namespace naoqi
{
namespace action
{

/** Checks a RunLed goal before it reaches ALLeds. An empty string means valid. */
inline std::string validateLedGoal(const naoqi_bridge_msgs::action::RunLed::Goal & goal)
{
  using Goal = naoqi_bridge_msgs::action::RunLed::Goal;
  auto in_unit_range = [](float v) {return std::isfinite(v) && v >= 0.f && v <= 1.f;};

  if (goal.mode > Goal::MODE_OFF) {
    return "unknown mode " + std::to_string(goal.mode);
  }
  if ((goal.mode == Goal::MODE_SET_INTENSITY || goal.mode == Goal::MODE_FADE_INTENSITY) &&
    !in_unit_range(goal.intensity))
  {
    return "intensity must be in [0, 1]";
  }
  if ((goal.mode == Goal::MODE_RGB || goal.mode == Goal::MODE_RGB_FADE) &&
    !(in_unit_range(goal.color.r) && in_unit_range(goal.color.g) && in_unit_range(goal.color.b)))
  {
    return "color components must be in [0, 1]";
  }
  if ((goal.mode == Goal::MODE_FADE_INTENSITY || goal.mode == Goal::MODE_RGB_FADE) &&
    !(std::isfinite(goal.duration) && goal.duration >= 0.f))
  {
    return "duration must be a non-negative number of seconds";
  }
  return "";
}

}  // namespace action
}  // namespace naoqi

#endif
