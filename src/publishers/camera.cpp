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
#include "camera.hpp"

/*
* ALDEBARAN includes
*/
#include "tools/alvisiondefinitions.h" // for kTop...

namespace naoqi
{
namespace publisher
{

CameraPublisher::CameraPublisher(const std::string & topic, int camera_source)
: topic_(topic),
  is_initialized_(false),
  camera_source_(camera_source)
{
}

CameraPublisher::~CameraPublisher()
= default;

void CameraPublisher::publish(
  const sensor_msgs::msg::Image::SharedPtr & img,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  pub_.publish(*img, camera_info);
}

void CameraPublisher::reset(rclcpp::Node * node)
{
  pub_ = image_transport::create_camera_publisher(node, topic_);
  // TODO: choose the QoS, or disable unwanted image_transport plugins in the launch file.

  is_initialized_ = true;
}

}  // namespace publisher
}  // namespace naoqi
