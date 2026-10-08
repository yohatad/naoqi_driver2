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

#include "heartbeat.hpp"

#include <exception>

namespace naoqi
{

Heartbeat::Heartbeat(
  rclcpp::Node * node, const qi::SessionPtr & session, std::chrono::milliseconds period)
: logger_(node->get_logger()),
  memory_(session->service("ALMemory").value()),
  callback_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
  timer_ = node->create_wall_timer(period, [this] {beat();}, callback_group_);
}

Heartbeat::~Heartbeat()
{
  timer_->cancel();
  // If this fails (the network is gone) the counter goes stale instead, and the
  // watchdog stops the base: also the right outcome.
  try {
    if (pending_.isValid()) {
      pending_.wait(500);  // so that the last beat cannot land after kStopped
    }
    memory_.async<void>("insertData", kKey, kStopped).wait(500);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger_, "Could not mark the heartbeat stopped: %s", e.what());
  }
}

void Heartbeat::beat()
{
  if (pending_.isValid()) {
    if (pending_.isRunning()) {
      return;
    }
    if (pending_.hasError(0)) {
      if (!failing_) {
        RCLCPP_WARN(
          logger_, "Writing the heartbeat to ALMemory failed (%s); a robot watchdog will "
          "stop the base", pending_.error(0).c_str());
      }
      failing_ = true;
    } else {
      failing_ = false;
    }
  }
  pending_ = memory_.async<void>("insertData", kKey, ++count_);
}

}  // namespace naoqi
