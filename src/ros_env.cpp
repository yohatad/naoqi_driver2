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

#include <naoqi_driver/ros_helpers.hpp>
#include "ros_env.hpp"
#include <stdexcept>

namespace naoqi
{
namespace ros_env
{

// Define static variables in implementation file
static std::string g_prefix = "";

std::string getROSIP(std::string network_interface)
{
  if (network_interface.empty()) {
    network_interface = "eth0";
  }

  using Map_IP = std::map<std::string, std::vector<std::string>>;
  Map_IP map_ip = static_cast<Map_IP>(qi::os::hostIPAddrs());

  if (map_ip.find(network_interface) == map_ip.end() ) {
    std::string available;
    for (const auto & entry : map_ip) {
      available += entry.first + " ";
    }
    RCLCPP_ERROR_STREAM(
      naoqi::helpers::Node::get_logger(), "Could not find network interface named " << network_interface <<
        ", possible interfaces are: " << available);
    throw std::runtime_error("Network interface '" + network_interface + "' not found");
  }

  return map_ip[network_interface][0];
}

void setPrefix(std::string s)
{
  g_prefix = s;
  RCLCPP_INFO_STREAM(naoqi::helpers::Node::get_logger(), "set prefix successfully to " << g_prefix);
}

std::string getPrefix()
{
  return g_prefix;
}

}  // namespace ros_env
}  // namespace naoqi
