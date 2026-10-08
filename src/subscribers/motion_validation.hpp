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

#ifndef MOTION_VALIDATION_HPP
#define MOTION_VALIDATION_HPP

#include <cmath>
#include <string>
#include <vector>

namespace naoqi
{
namespace subscriber
{
namespace validation
{

/** Checks run before a command reaches ALMotion. An empty string means valid. */

inline bool allFinite(const std::vector<float>& values)
{
  for (float v : values)
    if (!std::isfinite(v))
      return false;
  return true;
}

inline std::string validateVelocity(double x, double y, double theta)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta))
    return "velocity contains NaN or infinity";
  return "";
}

inline std::string validateJointAngles(const std::vector<std::string>& names,
                                       const std::vector<float>& angles,
                                       float speed)
{
  if (names.empty() || angles.empty())
    return "joint names and angles must not be empty";
  if (!allFinite(angles))
    return "joint angles contain NaN or infinity";
  if (!std::isfinite(speed) || speed <= 0.f || speed > 1.f)
    return "speed must be a fraction of max speed in (0, 1]";
  return "";
}

/**
 * @brief Validates a JointAnglesTrajectory. angles/times are flat arrays holding
 * the same number of waypoints for each name, in name order.
 */
inline std::string validateTrajectory(const std::vector<std::string>& names,
                                      const std::vector<float>& angles,
                                      const std::vector<float>& times,
                                      bool is_absolute,
                                      bool use_bezier)
{
  if (names.empty() || angles.empty() || times.empty())
    return "joint names, angles and times must not be empty";
  if (angles.size() != times.size())
    return "angles and times must have the same size";
  if (angles.size() % names.size() != 0)
    return "angles must hold the same number of waypoints for every joint";
  if (use_bezier && !is_absolute)
    return "relative angles are not supported with Bezier interpolation";
  if (!allFinite(angles) || !allFinite(times))
    return "angles or times contain NaN or infinity";

  const std::size_t waypoints = angles.size() / names.size();
  for (std::size_t joint = 0; joint < names.size(); ++joint)
  {
    float previous = 0.f;
    for (std::size_t wp = 0; wp < waypoints; ++wp)
    {
      const float t = times[joint * waypoints + wp];
      if (t <= previous)
        return "times must be positive and strictly increasing for each joint";
      previous = t;
    }
  }
  return "";
}

} // validation
} // subscriber
} // naoqi

#endif
