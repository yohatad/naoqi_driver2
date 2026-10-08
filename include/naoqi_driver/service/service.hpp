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

#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace service
{

/** @brief A ROS service backed by NAOqi calls. */
class Service
{
public:
  virtual ~Service() = default;

  /** Descriptive name. */
  virtual std::string name() const = 0;

  /** Service name in the ROS graph. */
  virtual std::string topic() const = 0;

  /** Creates (or re-creates) the service on the given node. */
  virtual void reset(rclcpp::Node * node) = 0;
};

}  // namespace service
}  // namespace naoqi

#endif
