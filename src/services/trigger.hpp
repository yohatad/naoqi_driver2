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

#ifndef TRIGGER_SERVICE_HPP
#define TRIGGER_SERVICE_HPP

#include <functional>
#include <memory>
#include <string>

#include <naoqi_driver/service/service.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace naoqi
{
namespace service
{

/**
 * A std_srvs/Trigger service around action: it answers success with the text action
 * returns, or failure with the message of the exception it throws.
 */
class TriggerService : public Service
{
public:
  TriggerService(
    const std::string & name, const std::string & topic, std::function<std::string()> action);

  std::string name() const override {return name_;}
  std::string topic() const override {return topic_;}
  void reset(rclcpp::Node * node) override;

private:
  void callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  const std::string name_;
  const std::string topic_;
  const std::function<std::string()> action_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

}  // namespace service
}  // namespace naoqi

#endif
