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

#ifndef CMD_VEL_WATCHDOG_HPP
#define CMD_VEL_WATCHDOG_HPP

#include <chrono>
#include <mutex>

namespace naoqi
{
namespace subscriber
{

/**
 * @brief Detects a /cmd_vel stream that went silent while the base was moving.
 *
 * ALMotion.move() keeps the last velocity until told otherwise, so if the
 * publisher dies the robot would keep driving. The owner feeds every command in
 * and polls expired(); it returns true once per silence so the base is stopped once.
 * A timeout of zero (or less) disables the watchdog.
 */
class CmdVelWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  explicit CmdVelWatchdog(std::chrono::nanoseconds timeout)
  : timeout_(timeout) {}

  bool enabled() const {return timeout_.count() > 0;}

  void onCommand(bool is_zero, Clock::time_point now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = now;
    moving_ = !is_zero;
  }

  bool expired(Clock::time_point now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled() || !moving_ || now - last_command_ < timeout_) {
      return false;
    }
    moving_ = false;
    return true;
  }

private:
  const std::chrono::nanoseconds timeout_;
  std::mutex mutex_;
  Clock::time_point last_command_;
  bool moving_ = false;
};

} // subscriber
} // naoqi

#endif
