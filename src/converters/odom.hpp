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

#ifndef ODOM_CONVERTER_HPP
#define ODOM_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>
#include <naoqi_driver/ros_helpers.hpp>

/*
* ROS includes
*/
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>

namespace naoqi
{
namespace converter
{

class OdomConverter : public BaseConverter<OdomConverter>
{

  typedef boost::function<void(nav_msgs::msg::Odometry&)> Callback_t;

public:
  OdomConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

  void registerCallback( message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

  void reset( );

  // Method to reset odometry offset (uses instance's motion proxy)
  void resetOdometry();
  
  // Static method to get current offsets (for debugging)
  static void getOffsets(float& x, float& y, float& z, float& wx, float& wy, float& wz);
  
  // Static method to set offsets directly (used by service)
  static void setOffsets(float x, float y, float z, float wx, float wy, float wz);

private:

  /** Motion Proxy **/
  qi::AnyObject p_motion_;

  std::map<message_actions::MessageAction, Callback_t> callbacks_;
  nav_msgs::msg::Odometry msg_;

  // Static offset variables for software-based odometry reset.
  // Written from the /reset_odom service thread and read on the publish
  // thread, so every access must hold offsets_mutex_.
  static std::mutex offsets_mutex_;
  static float position_offset_x_;
  static float position_offset_y_;
  static float position_offset_z_;
  static float orientation_offset_wx_;
  static float orientation_offset_wy_;
  static float orientation_offset_wz_;
}; // class

} //publisher
} // naoqi

#endif
