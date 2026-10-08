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

#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace subscriber
{

/**
 * @brief A ROS subscription that forwards incoming commands to NAOqi.
 * Implemented through subscriber::BaseSubscriber.
 */
class Subscriber
{
public:
  virtual ~Subscriber() = default;

  /** Descriptive name, unique among the driver's subscribers. */
  virtual std::string name() const = 0;

  /** Topic subscribed to. */
  virtual std::string topic() const = 0;

  virtual bool isInitialized() const = 0;

  /** Creates (or re-creates) the subscription on the given node. */
  virtual void reset(rclcpp::Node * node) = 0;
};

}  // namespace subscriber
}  // namespace naoqi

#endif
