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
#include "joint_state.hpp"
#include "nao_footprint.hpp"
#include "odom.hpp"

/*
* ROS includes
*/
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace naoqi
{

namespace converter
{

JointStateConverter::JointStateConverter( const std::string& name, const float& frequency, const BufferPtr& tf2_buffer, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_motion_( session->service("ALMotion").value() ),
  p_memory_( session->service("ALMemory").value() ),
  tf2_buffer_(tf2_buffer)
{
}

JointStateConverter::~JointStateConverter()
{
  // clear all sessions
}

void JointStateConverter::reset()
{
  std::string robot_desc = naoqi::tools::getRobotDescription(robot_);
  if ( robot_desc.empty() )
  {
    std::cout << "error in loading robot description" << std::endl;
    return;
  }

  urdf::Model model;
  model.initString(robot_desc);
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel( model, tree );

  addChildren( tree.getRootSegment() );

  // store the static Tibia -> base_footprint offset from the URDF, so that
  // base_footprint -> base_link can be derived from the current leg kinematics
  tibia_to_footprint_offset_.setIdentity();
  std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator footprint_seg = segments_fixed_.find("base_footprint_joint");
  if ( footprint_seg != segments_fixed_.end() )
  {
    KDL::Frame frame = footprint_seg->second.segment.pose(0);
    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    tibia_to_footprint_offset_.setOrigin( tf2::Vector3(frame.p.x(), frame.p.y(), frame.p.z()) );
    tibia_to_footprint_offset_.setRotation( tf2::Quaternion(qx, qy, qz, qw) );
  }
  else
  {
    RCLCPP_ERROR(helpers::Node::get_logger(), "base_footprint_joint not found in URDF, base_footprint -> base_link will be identity");
  }

  // whether to also publish odom -> base_footprint from wheel odometry
  // (off by default: an external localization source such as FAST-LIO owns this edge)
  const auto& node = helpers::Node::get_node();
  if ( !node->has_parameter("publish_wheel_odom_tf") )
  {
    node->declare_parameter<bool>("publish_wheel_odom_tf", false);
  }
  node->get_parameter("publish_wheel_odom_tf", publish_wheel_odom_tf_);

  // set mimic joint list
  mimic_.clear();
  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++){
    if(i->second->mimic){
      mimic_.insert(make_pair(i->first, i->second->mimic));
    }
  }
  // pre-fill joint states message
  msg_joint_states_.name = p_motion_.call<std::vector<std::string> >("getBodyNames", "Body" );
}

void JointStateConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void JointStateConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
  // get joint state values
  std::vector<double> al_joint_angles = p_motion_.call<std::vector<double> >("getAngles", "Body", true );
  std::vector<double> al_joint_velocities;
  std::vector<double> al_joint_torques;

  const rclcpp::Time& stamp = helpers::Time::now();

  /**
   * JOINT STATE PUBLISHER
   */
  msg_joint_states_.header.stamp = stamp;

  // STUPID CONVERTION FROM FLOAT TO DOUBLE ARRAY --> OPTIMIZE THAT!
  msg_joint_states_.position = std::vector<double>( al_joint_angles.begin(), al_joint_angles.end() );

  /**
   * ROBOT STATE PUBLISHER
   */
  // put joint states in tf broadcaster
  std::map< std::string, double > joint_state_map;
  // stupid version --> change this with std::transform c++11 ??!
//  std::transform( msg_joint_states_.name.begin(), msg_joint_states_.name.end(), msg_joint_states_.position.begin(),
//                  std::inserter( joint_state_map, joint_state_map.end() ),
//                  std::make_pair);
  // Build joint_state_map first
  std::vector<double>::const_iterator itPos = msg_joint_states_.position.begin();
  for(std::vector<std::string>::const_iterator itName = msg_joint_states_.name.begin();
      itName != msg_joint_states_.name.end();
      ++itName, ++itPos)
  {
    joint_state_map[*itName] = *itPos;
  }

  // Batch fetch velocity and torque - single network call
  std::vector<std::string> all_keys;
  all_keys.reserve(msg_joint_states_.name.size() * 2);
  for (const auto& name : msg_joint_states_.name) {
    all_keys.push_back("Motion/Velocity/Sensor/" + name);
    all_keys.push_back("Motion/Torque/Sensor/" + name);
  }

  try {
    std::vector<qi::AnyValue> all_values = p_memory_.call<std::vector<qi::AnyValue>>("getListData", all_keys);
    
    // Unpack: even indices = velocity, odd indices = torque
    for (size_t i = 0; i < msg_joint_states_.name.size(); ++i) {
      try {
        al_joint_velocities.push_back(all_values[i * 2].toDouble());
        al_joint_torques.push_back(all_values[i * 2 + 1].toDouble());
      } catch (...) {
        al_joint_velocities.push_back(std::numeric_limits<double>::quiet_NaN());
        al_joint_torques.push_back(std::numeric_limits<double>::quiet_NaN());
      }
    }
  } catch (qi::FutureUserException& e) {
    // Fallback: fill with NaN if batch call fails
    for (size_t i = 0; i < msg_joint_states_.name.size(); ++i) {
      al_joint_velocities.push_back(std::numeric_limits<double>::quiet_NaN());
      al_joint_torques.push_back(std::numeric_limits<double>::quiet_NaN());
    }
  }

  msg_joint_states_.velocity = al_joint_velocities;
  msg_joint_states_.effort = al_joint_torques;

  // for mimic map
  for(MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++){
    if(joint_state_map.find(i->second->joint_name) != joint_state_map.end()){
      double pos = joint_state_map[i->second->joint_name] * i->second->multiplier + i->second->offset;
      joint_state_map[i->first] = pos;
    }
  }

  // reset the transforms we want to use at this time
  tf_transforms_.clear();
  static const std::string& jt_tf_prefix = "";
  setTransforms(joint_state_map, stamp, jt_tf_prefix);
  setFixedTransforms(jt_tf_prefix, stamp);

  const rclcpp::Time &odom_stamp = stamp;

  // Optionally publish odom -> base_footprint from wheel odometry. Leave disabled
  // when an external localization source (e.g. FAST-LIO) owns this edge.
  if ( publish_wheel_odom_tf_ )
  {
    std::vector<float> al_odometry_data = p_motion_.call<std::vector<float> >( "getRobotPosition", true );

    // Get the current offsets from OdomConverter
    float offset_x, offset_y, offset_z, offset_wx, offset_wy, offset_wz;
    naoqi::converter::OdomConverter::getOffsets(offset_x, offset_y, offset_z, offset_wx, offset_wy, offset_wz);

    // Extract 2D position data
    float raw_x = al_odometry_data[0];      // X position in world frame
    float raw_y = al_odometry_data[1];      // Y position in world frame
    float raw_theta = al_odometry_data[2];  // Yaw angle in world frame [-pi, pi]

    // Transform position to robot's initial frame
    float cos_offset = std::cos(-offset_wz);
    float sin_offset = std::sin(-offset_wz);

    float dx = raw_x - offset_x;
    float dy = raw_y - offset_y;

    // Rotate by inverse of initial orientation
    const float odomX = dx * cos_offset - dy * sin_offset;
    const float odomY = dx * sin_offset + dy * cos_offset;
    const float odomZ = 0.0f;  // Ground level

    // Normalize theta to robot's initial orientation
    const float odomTheta = raw_theta - offset_wz;

    // Create quaternion from yaw angle (2D rotation)
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0.0, 0.0, odomTheta);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg( tf_quat );

    // Publish odom → base_footprint (ground level)
    geometry_msgs::msg::TransformStamped msg_tf_odom;
    msg_tf_odom.header.frame_id = "pepper_odom";
    msg_tf_odom.child_frame_id = "base_footprint";
    msg_tf_odom.header.stamp = odom_stamp;

    msg_tf_odom.transform.translation.x = odomX;
    msg_tf_odom.transform.translation.y = odomY;
    msg_tf_odom.transform.translation.z = odomZ;
    msg_tf_odom.transform.rotation = odom_quat;

    tf_transforms_.push_back( msg_tf_odom );
    tf2_buffer_->setTransform( msg_tf_odom, "naoqiconverter", false);
  }

  // Connect base_footprint to base_link (bridges the external odometry frame to the robot body)
  // Derived from the current leg kinematics (HipRoll/HipPitch/KneePitch), via the
  // base_link -> Tibia transform computed by setTransforms() above, combined with
  // the static Tibia -> base_footprint offset from the URDF. This keeps the
  // odom -> base_footprint -> base_link chain consistent as the legs move,
  // instead of assuming a fixed default-pose height.
  static geometry_msgs::msg::TransformStamped msg_tf_base;
  msg_tf_base.header.frame_id = "base_footprint";
  msg_tf_base.child_frame_id = "base_link";
  msg_tf_base.header.stamp = odom_stamp;

  try {
    geometry_msgs::msg::TransformStamped tf_base_link_to_tibia =
        tf2_buffer_->lookupTransform("base_link", "Tibia", tf2::TimePointZero);

    tf2::Transform t_base_link_to_tibia;
    tf2::fromMsg(tf_base_link_to_tibia.transform, t_base_link_to_tibia);

    tf2::Transform t_base_link_to_footprint = t_base_link_to_tibia * tibia_to_footprint_offset_;
    msg_tf_base.transform = tf2::toMsg(t_base_link_to_footprint.inverse());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(helpers::Node::get_logger(), "Could not compute base_footprint -> base_link: %s", ex.what());
    msg_tf_base.transform.rotation.w = 1.0;
  }

  tf_transforms_.push_back( msg_tf_base );
  tf2_buffer_->setTransform( msg_tf_base, "naoqiconverter", true);

  if (robot_ == robot::NAO )
  {
    nao::addBaseFootprint( tf2_buffer_, tf_transforms_, odom_stamp - tf2::durationFromSec(0.1) );
  }

  for( message_actions::MessageAction action: actions )
  {
    callbacks_[action]( msg_joint_states_, tf_transforms_ );
  }
}


// Copied from robot state publisher
void JointStateConverter::setTransforms(const std::map<std::string, double>& joint_positions, const rclcpp::Time& time, const std::string& tf_prefix)
{
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;

  // loop over all joints
  for (std::map<std::string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
    std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()){
      seg->second.segment.pose(jnt->second).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                            tf_transform.transform.rotation.y,
                                                            tf_transform.transform.rotation.z,
                                                            tf_transform.transform.rotation.w);
      tf_transform.transform.translation.x = seg->second.segment.pose(jnt->second).p.x();
      tf_transform.transform.translation.y = seg->second.segment.pose(jnt->second).p.y();
      tf_transform.transform.translation.z = seg->second.segment.pose(jnt->second).p.z();

      //tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
      //tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
      tf_transform.header.frame_id = seg->second.root; // tf2 does not suppport tf_prefixing
      tf_transform.child_frame_id = seg->second.tip;

      tf_transforms_.push_back(tf_transform);

      if (tf2_buffer_)
          tf2_buffer_->setTransform(tf_transform, "naoqiconverter", false);
    }
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

// Copied from robot state publisher
void JointStateConverter::setFixedTransforms(const std::string& tf_prefix, const rclcpp::Time& time)
{
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;

  // loop over all fixed segments
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
    // Skip Tibia → base_footprint joint to avoid TF loop
    // We publish odom → base_footprint → base_link manually
    if (seg->second.root == "Tibia" && seg->second.tip == "base_footprint") {
      continue;
    }

    seg->second.segment.pose(0).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                tf_transform.transform.rotation.y,
                                                tf_transform.transform.rotation.z,
                                                tf_transform.transform.rotation.w);
    tf_transform.transform.translation.x = seg->second.segment.pose(0).p.x();
    tf_transform.transform.translation.y = seg->second.segment.pose(0).p.y();
    tf_transform.transform.translation.z = seg->second.segment.pose(0).p.z();

    //tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
    //tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
    tf_transform.header.frame_id = seg->second.root;
    tf_transform.child_frame_id = seg->second.tip;

    tf_transforms_.push_back(tf_transform);

    if (tf2_buffer_)
      tf2_buffer_->setTransform(tf_transform, "naoqiconverter", true);
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

void JointStateConverter::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (unsigned int i=0; i<children.size(); i++){
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    robot_state_publisher::SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None){
      segments_fixed_.insert(std::make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(helpers::Node::get_logger(), "Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else{
      segments_.insert(std::make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(helpers::Node::get_logger(), "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

} //publisher
} // naoqi
