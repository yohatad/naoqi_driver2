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

#include "odom_reset.hpp"
#include <iostream>
#include "../helpers/driver_helpers.hpp"
#include "../converters/odom.hpp"

namespace naoqi
{
namespace service
{

OdomResetService::OdomResetService( const std::string& name, const std::string& topic, const qi::SessionPtr& session )
  : name_(name),
  topic_(topic),
  session_(session)
{}

void OdomResetService::reset( rclcpp::Node* node )
{
  service_ = node->create_service<std_srvs::srv::Trigger>(
    topic_,
    std::bind(&OdomResetService::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void OdomResetService::callback( const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> resp )
{
  std::cout << "Receiving service call to reset odometry" << std::endl;
  
  try {
    // Get the motion service
    qi::AnyObject p_motion = session_->service("ALMotion").value();
    
    // Get current ground pose to set as offset. Must be the same API and
    // frame as OdomConverter::callAll: getRobotPosition returns [X, Y, Theta].
    bool use_sensor = true;
    std::vector<float> current_pos = p_motion.call<std::vector<float> >( "getRobotPosition", use_sensor );

    // Set offsets to current position, so future readings will be relative to this point
    naoqi::converter::OdomConverter::setOffsets(
      current_pos[0],
      current_pos[1],
      0.0f,
      0.0f,
      0.0f,
      current_pos[2]
    );

    resp->success = true;
    resp->message = "Odometry reset successfully using software offset";
    std::cout << "Odometry reset successfully" << std::endl;
    std::cout << "New offsets - X: " << current_pos[0]
              << " Y: " << current_pos[1]
              << " Theta: " << current_pos[2] << std::endl;
  } 
  catch (const std::exception& e) {
    resp->success = false;
    resp->message = std::string("Failed to reset odometry: ") + e.what();
    std::cerr << "Failed to reset odometry: " << e.what() << std::endl;
  }
}

}
}
