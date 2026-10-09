/*
 * Copyright 2017 Aldebaran
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

#ifndef GET_LANGUAGE_SERVICE_HPP
#define GET_LANGUAGE_SERVICE_HPP

#include <naoqi_driver/service/service.hpp>

#include <rclcpp/rclcpp.hpp>

#include <naoqi_bridge_msgs/srv/get_string.hpp>
#include <qi/session.hpp>

namespace naoqi
{
namespace service
{

class GetLanguageService : public Service
{
public:
  GetLanguageService(
    const std::string & name, const std::string & topic,
    const qi::SessionPtr & session);

  ~GetLanguageService() = default;

  std::string name() const override
  {
    return name_;
  }

  std::string topic() const override
  {
    return topic_;
  }

  void reset(rclcpp::Node * node) override;

  void callback(
    const std::shared_ptr<naoqi_bridge_msgs::srv::GetString::Request> req,
    std::shared_ptr<naoqi_bridge_msgs::srv::GetString::Response> resp);

private:
  const std::string name_;
  const std::string topic_;

  const qi::SessionPtr & session_;
  rclcpp::Service<naoqi_bridge_msgs::srv::GetString>::SharedPtr service_;
};

}  // namespace service
}  // namespace naoqi
#endif
