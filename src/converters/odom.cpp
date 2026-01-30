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
#include "odom.hpp"
#include "../tools/from_any_value.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace naoqi
{
namespace converter
{

// Initialize static offset variables
float OdomConverter::position_offset_x_ = 0.0f;
float OdomConverter::position_offset_y_ = 0.0f;
float OdomConverter::position_offset_z_ = 0.0f;
float OdomConverter::orientation_offset_wx_ = 0.0f;
float OdomConverter::orientation_offset_wy_ = 0.0f;
float OdomConverter::orientation_offset_wz_ = 0.0f;

OdomConverter::OdomConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_motion_( session->service("ALMotion").value() )

{
}

void OdomConverter::registerCallback( message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void OdomConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{

  int FRAME_WORLD = 1;
  bool use_sensor = true;
  // documentation of getPosition available here: http://doc.aldebaran.com/2-1/naoqi/motion/control-cartesian.html
  std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getPosition", "Torso", FRAME_WORLD, use_sensor );

  const rclcpp::Time& odom_stamp = helpers::Time::now();
  std::vector<float> al_speed_data = p_motion_.call<std::vector<float> >( "getRobotVelocity" );

  // Apply offsets to raw sensor data
  const float& odomX  =  al_odometry_data[0] - position_offset_x_;
  const float& odomY  =  al_odometry_data[1] - position_offset_y_;
  const float& odomZ  =  al_odometry_data[2] - position_offset_z_;
  const float& odomWX =  al_odometry_data[3] - orientation_offset_wx_;
  const float& odomWY =  al_odometry_data[4] - orientation_offset_wy_;
  const float& odomWZ =  al_odometry_data[5] - orientation_offset_wz_;

  const float& dX = al_speed_data[0];
  const float& dY = al_speed_data[1];
  const float& dWZ = al_speed_data[2];

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion tf_quat;
  tf_quat.setRPY( odomWX, odomWY, odomWZ );
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg( tf_quat );

  static nav_msgs::msg::Odometry msg_odom;
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "base_link";
  msg_odom.header.stamp = odom_stamp;

  msg_odom.pose.pose.orientation = odom_quat;
  msg_odom.pose.pose.position.x = odomX;
  msg_odom.pose.pose.position.y = odomY;
  msg_odom.pose.pose.position.z = odomZ;

  msg_odom.twist.twist.linear.x = dX;
  msg_odom.twist.twist.linear.y = dY;
  msg_odom.twist.twist.linear.z = 0;

  msg_odom.twist.twist.angular.x = 0;
  msg_odom.twist.twist.angular.y = 0;
  msg_odom.twist.twist.angular.z = dWZ;

  msg_odom.pose.covariance[0] = 0.01;
  msg_odom.pose.covariance[7] = 0.01;
  msg_odom.pose.covariance[35] = 0.1;
  msg_odom.twist.covariance[0] = 0.01;
  msg_odom.twist.covariance[7] = 0.01;
  msg_odom.twist.covariance[35] = 0.1;

  for( message_actions::MessageAction action: actions )
  {
    callbacks_[action](msg_odom);

  }
}

void OdomConverter::reset( )
{
  // Call the static reset method
  resetOdometry();
}

void OdomConverter::resetOdometry()
{
  try {
    // Get current position to set as offset
    std::vector<float> current_pos = p_motion_.call<std::vector<float> >( "getPosition", "Torso", 1, true );
    
    // Set offsets to current position, so future readings will be relative to this point
    position_offset_x_ = current_pos[0];
    position_offset_y_ = current_pos[1];
    position_offset_z_ = current_pos[2];
    orientation_offset_wx_ = current_pos[3];
    orientation_offset_wy_ = current_pos[4];
    orientation_offset_wz_ = current_pos[5];
    
    std::cout << "Odometry reset using software offset" << std::endl;
    std::cout << "New offsets - X: " << position_offset_x_ 
              << " Y: " << position_offset_y_ 
              << " Z: " << position_offset_z_ << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Failed to reset odometry: " << e.what() << std::endl;
  }
}

void OdomConverter::getOffsets(float& x, float& y, float& z, float& wx, float& wy, float& wz)
{
  x = position_offset_x_;
  y = position_offset_y_;
  z = position_offset_z_;
  wx = orientation_offset_wx_;
  wy = orientation_offset_wy_;
  wz = orientation_offset_wz_;
}

void OdomConverter::setOffsets(float x, float y, float z, float wx, float wy, float wz)
{
  position_offset_x_ = x;
  position_offset_y_ = y;
  position_offset_z_ = z;
  orientation_offset_wx_ = wx;
  orientation_offset_wy_ = wy;
  orientation_offset_wz_ = wz;
  
  std::cout << "Odometry offsets set to - X: " << position_offset_x_ 
            << " Y: " << position_offset_y_ 
            << " Z: " << position_offset_z_ << std::endl;
}

} //converter
} // naoqi
