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

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <naoqi_driver/ros_helpers.hpp>
#include <string>

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace publisher
{

/**
* @brief Publisher concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class Publisher
{

public:
  /**
  * @brief Constructor for publisher interface
  */
  template<typename T>
  Publisher(const T & pub)
  : pubPtr_(std::make_shared<PublisherModel<T>>(pub) )
  {}

  /**
  * @brief checks if the publisher is correctly initialized on the ros-master
  @ @return bool value indicating true for success
  */
  bool isInitialized() const
  {
    return pubPtr_->isInitialized();
  }

  /**
  * @brief checks if the publisher has a subscription and is hence allowed to publish
  * @return bool value indicating true for number of sub > 0
  */
  bool isSubscribed() const
  {
    return pubPtr_->isSubscribed();
  }

  /**
  * @brief initializes/resets the publisher into ROS with a given Node object,
  * this will be called at first for initialization
  * @param node rclcpp::Node pointer from which the publisher is created
  */
  void reset(rclcpp::Node * node)
  {
    RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), topic() << " is resetting");
    pubPtr_->reset(node);
    RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), topic() << " reset");
  }

  /**
  * @brief getting the topic to publish on
  * @return string indicating the topic
  */
  std::string topic() const
  {
    return pubPtr_->topic();
  }

  friend bool operator==(const Publisher & lhs, const Publisher & rhs)
  {
    // decision made for OR-comparison since we want to be more restrictive
    return lhs.topic() == rhs.topic();
  }

  friend bool operator==(
    const std::shared_ptr<Publisher> & lhs,
    const std::shared_ptr<Publisher> & rhs)
  {
    return operator==(*lhs, *rhs);
  }

private:
  /**
  * BASE concept struct
  */
  struct PublisherConcept
  {
    virtual ~PublisherConcept() = default;
    virtual bool isInitialized() const = 0;
    virtual bool isSubscribed() const = 0;
    virtual void reset(rclcpp::Node * node) = 0;
    virtual std::string topic() const = 0;
  };

  /**
  * templated instances of base concept
  */
  template<typename T>
  struct PublisherModel : public PublisherConcept
  {
    PublisherModel(const T & other)
    : publisher_(other)
    {}

    std::string topic() const override
    {
      return publisher_->topic();
    }

    bool isInitialized() const override
    {
      return publisher_->isInitialized();
    }

    bool isSubscribed() const override
    {
      return publisher_->isSubscribed();
    }

    void reset(rclcpp::Node * node) override
    {
      publisher_->reset(node);
    }

    T publisher_;
  };

  std::shared_ptr<PublisherConcept> pubPtr_;

}; // class publisher

}  // namespace publisher
}  // namespace naoqi

#endif
