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

#ifndef PARAMETER_HELPERS_HPP
#define PARAMETER_HELPERS_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace helpers
{

/**
 * Declares (once) and reads a numeric parameter. It is dynamically typed, so both
 * "0" and "0.5" are accepted from the command line or a launch file.
 */
inline double declareNumberParameter(
  rclcpp::Node * node, const std::string & name, double default_value,
  const std::string & description)
{
  if (!node->has_parameter(name)) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.dynamic_typing = true;
    node->declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  }
  const rclcpp::Parameter parameter = node->get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return parameter.as_double();
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  RCLCPP_WARN(node->get_logger(), "%s must be a number, using %g", name.c_str(), default_value);
  return default_value;
}

}  // namespace helpers
}  // namespace naoqi

#endif
