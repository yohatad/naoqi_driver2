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

#ifndef JOINT_STATES_PUBLISHER_HPP
#define JOINT_STATES_PUBLISHER_HPP

/*
* ROS includes
*/
#include <naoqi_driver/publisher/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace naoqi
{
namespace publisher
{

class JointStatePublisher : public Publisher
{

public:
  JointStatePublisher(const std::string & topic = "/joint_states");

  std::string topic() const override
  {
    return topic_;
  }

  bool isInitialized() const override
  {
    return is_initialized_;
  }

  virtual void publish(
    const sensor_msgs::msg::JointState & js_msg,
    const std::vector<geometry_msgs::msg::TransformStamped> & tf_transforms);

  void reset(rclcpp::Node * node) override;

  bool isSubscribed() const override;

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcasterPtr_;

  /** initialize separate publishers for js and odom */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  std::string topic_;

  bool is_initialized_;

}; // class

}  // namespace publisher
}  // namespace naoqi

#endif
