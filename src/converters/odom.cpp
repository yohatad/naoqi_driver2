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
  bool use_sensor = true;

  // Get robot base position in world frame (X, Y, Theta)
  std::vector<float> al_position_data = p_motion_.call<std::vector<float> >( "getRobotPosition", use_sensor );

  const rclcpp::Time& odom_stamp = helpers::Time::now();
  std::vector<float> al_speed_data = p_motion_.call<std::vector<float> >( "getRobotVelocity" );

  // Raw data from NAOqi
  float raw_x = al_position_data[0];      // X position in world frame
  float raw_y = al_position_data[1];      // Y position in world frame
  float raw_theta = al_position_data[2];  // Yaw angle in world frame [-pi, pi]

  // DEBUG: Print raw values and offsets every 50 cycles (about once per second at 50Hz)
  static int debug_counter = 0;
  if (++debug_counter >= 50) {
    std::cout << "[ODOM DEBUG] Raw: X=" << raw_x << " Y=" << raw_y << " Theta=" << raw_theta << std::endl;
    std::cout << "[ODOM DEBUG] Offsets: X=" << position_offset_x_ << " Y=" << position_offset_y_ << " Theta=" << orientation_offset_wz_ << std::endl;
    debug_counter = 0;
  }

  // Transform position to robot's initial frame
  // This makes the starting position (0, 0) and starting orientation 0
  float cos_offset = std::cos(-orientation_offset_wz_);
  float sin_offset = std::sin(-orientation_offset_wz_);

  float dx = raw_x - position_offset_x_;
  float dy = raw_y - position_offset_y_;

  // Rotate delta by inverse of initial orientation
  const float odomX = dx * cos_offset - dy * sin_offset;
  const float odomY = dx * sin_offset + dy * cos_offset;
  const float odomZ = 0.0f;  // 2D navigation - keep at 0

  // Normalize theta to robot's initial orientation
  const float odomTheta = raw_theta - orientation_offset_wz_;

  // Velocity is already in FRAME_ROBOT (base_link) - perfect for odometry
  const float& dX = al_speed_data[0];
  const float& dY = al_speed_data[1];
  const float& dWZ = al_speed_data[2];

  // Create quaternion from yaw angle
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, odomTheta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg( tf_quat );

  static nav_msgs::msg::Odometry msg_odom;
  msg_odom.header.frame_id = "odom";
  msg_odom.child_frame_id = "base_footprint";  // Changed to match JointStateConverter TF
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
    // Get current position using same API as odometry measurement
    bool use_sensor = true;
    std::vector<float> current_pos = p_motion_.call<std::vector<float> >( "getRobotPosition", use_sensor );

    // getRobotPosition returns 3 values: [X, Y, Theta]
    position_offset_x_ = current_pos[0];
    position_offset_y_ = current_pos[1];
    position_offset_z_ = 0.0f;  // 2D navigation - always 0
    orientation_offset_wx_ = 0.0f;  // 2D - no roll
    orientation_offset_wy_ = 0.0f;  // 2D - no pitch
    orientation_offset_wz_ = current_pos[2];  // Yaw/Theta

    std::cout << "Odometry reset using software offset" << std::endl;
    std::cout << "New offsets - X: " << position_offset_x_
              << " Y: " << position_offset_y_
              << " Theta: " << orientation_offset_wz_ << std::endl;
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
