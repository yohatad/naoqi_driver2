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

/*
 * LOCAL includes
 */
#include <naoqi_driver/ros_helpers.hpp>
#include <memory>
#include <functional>
#include "moveto.hpp"

/*
 * ROS includes
 */
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

MovetoSubscriber::MovetoSubscriber(
  const std::string & name, const std::string & topic, const qi::SessionPtr & session,
  const std::shared_ptr<tf2_ros::Buffer> & tf2_buffer)
: BaseSubscriber(name, topic, session),
  p_motion_(session->service("ALMotion").value() ),
  tf2_buffer_(tf2_buffer)
{}

void MovetoSubscriber::reset(rclcpp::Node * node)
{
  sub_moveto_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_,
    10,
    std::bind(&MovetoSubscriber::callback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void MovetoSubscriber::callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
  if (pose_msg->header.frame_id == "odom") {
    geometry_msgs::msg::PoseStamped pose_msg_bf;

    try {
      // Use the latest transform without waiting: this runs on the driver's only
      // loop thread, and blocking here would stall every sensor stream.
      const auto odom_to_base = tf2_buffer_->lookupTransform(
        "base_footprint", "odom", tf2::TimePointZero);
      tf2::doTransform(*pose_msg, pose_msg_bf, odom_to_base);

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);

      RCLCPP_INFO_STREAM(
        naoqi::helpers::Node::get_logger(),
        "odom to move x: " << pose_msg_bf.pose.position.x << " y: " << pose_msg_bf.pose.position.y << " yaw: " <<
          yaw);

      if (std::isnan(yaw)) {
        yaw = 0.0;
        RCLCPP_WARN_STREAM(naoqi::helpers::Node::get_logger(), "Yaw is nan, changed to 0.0");
      }

      p_motion_.async<void>(
        "moveTo",
        pose_msg_bf.pose.position.x,
        pose_msg_bf.pose.position.y,
        yaw);

    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_STREAM(
        naoqi::helpers::Node::get_logger(),
        "Cannot transform the moveto goal from odom to base_footprint: " << e.what());
    }
  } else if (pose_msg->header.frame_id == "base_footprint") {
    double yaw = helpers::transform::getYaw(pose_msg->pose);
    RCLCPP_INFO_STREAM(
      naoqi::helpers::Node::get_logger(),
      "going to move x: " << pose_msg->pose.position.x << " y: " << pose_msg->pose.position.y << " yaw: " <<
        yaw);

    if (std::isnan(yaw)) {
      yaw = 0.0;
      RCLCPP_WARN_STREAM(naoqi::helpers::Node::get_logger(), "Yaw is nan, changed to 0.0");
    }

    p_motion_.async<void>(
      "moveTo",
      pose_msg->pose.position.x,
      pose_msg->pose.position.y,
      yaw);
  } else {
    RCLCPP_WARN_STREAM(
      naoqi::helpers::Node::get_logger(), "Cannot reach position expressed in the " << pose_msg->header.frame_id << " frame, enter a valid frame id in the pose's header"
        " (base_footprint or odom)");
  }
}

}  // namespace subscriber
}  // namespace naoqi
