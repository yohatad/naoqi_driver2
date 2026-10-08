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

#include "trigger.hpp"

#include <exception>
#include <utility>

namespace naoqi
{
namespace service
{

TriggerService::TriggerService(
  const std::string & name, const std::string & topic, std::function<std::string()> action)
: name_(name),
  topic_(topic),
  action_(std::move(action))
{}

void TriggerService::reset(rclcpp::Node * node)
{
  service_ = node->create_service<std_srvs::srv::Trigger>(
    topic_,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      callback(request, response);
    });
}

void TriggerService::callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  try {
    response->message = action_();
    response->success = true;
  } catch (const std::exception & e) {
    response->message = e.what();
    response->success = false;
  }
}

}  // namespace service
}  // namespace naoqi
