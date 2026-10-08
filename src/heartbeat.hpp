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

#ifndef HEARTBEAT_HPP
#define HEARTBEAT_HPP

#include <chrono>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>
#include <rclcpp/rclcpp.hpp>

namespace naoqi
{

/**
 * Writes an increasing counter to ALMemory every period, from the node's callback
 * threads, and kStopped when destroyed. scripts/robot_watchdog.py runs on the robot and
 * stops the base when the counter stops changing: the driver died, hangs or lost the
 * network, so nothing would stop the last velocity sent to ALMotion.move().
 */
class Heartbeat
{
public:
  static constexpr const char * kKey = "NaoqiDriver/Heartbeat";
  /** Written on a clean exit: the driver stopped the robot itself. */
  static constexpr int kStopped = -1;

  Heartbeat(rclcpp::Node * node, const qi::SessionPtr & session, std::chrono::milliseconds period);
  ~Heartbeat();

  Heartbeat(const Heartbeat &) = delete;
  Heartbeat & operator=(const Heartbeat &) = delete;

private:
  void beat();

  rclcpp::Logger logger_;
  qi::AnyObject memory_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  // The last write. While it is pending (network stalled) no new one is queued.
  qi::Future<void> pending_;
  bool failing_ = false;  // warn once per outage, not on every beat
  int count_ = 0;
};

}  // namespace naoqi

#endif
