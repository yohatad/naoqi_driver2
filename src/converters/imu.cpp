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

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

/*
* LOCAL includes
*/
#include <naoqi_driver/ros_helpers.hpp>
#include "imu.hpp"
#include "tools/from_any_value.hpp"

/*
* ROS includes
*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace naoqi
{
namespace converter
{

ImuConverter::ImuConverter(
  const std::string & name, const IMU::Location & location,
  float frequency, const qi::SessionPtr & session)
: BaseConverter(name, frequency, session),
  p_memory_(session->service("ALMemory").value())
{
  if (location == IMU::TORSO) {
    msg_imu_.header.frame_id = "base_link";
    data_names_list_.push_back("DCM/Time");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AngleZ/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
  } else if (location == IMU::BASE) {
    msg_imu_.header.frame_id = "base_footprint";
    data_names_list_.push_back("DCM/Time");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AngleZ/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/GyroscopeZ/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerX/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerY/Sensor/Value");
    data_names_list_.push_back("Device/SubDeviceList/InertialSensorBase/AccelerometerZ/Sensor/Value");
  }

  // Pepper 1.8 has no torso IMU, though ALMemory keeps some of its keys: reading the
  // others would publish -1 and log a warning for each, on every sample. Throwing here
  // makes the driver disable this IMU.
  const auto available = p_memory_.call<std::vector<std::string>>("getDataListName");
  std::vector<std::string> missing;
  for (const auto & key : data_names_list_) {
    if (std::find(available.begin(), available.end(), key) == available.end()) {
      missing.push_back(key);
    }
  }
  if (!missing.empty()) {
    throw std::runtime_error(
            "this robot has no such IMU: " + std::to_string(missing.size()) +
            " of its ALMemory keys are missing, such as " + missing.front());
  }
}

ImuConverter::~ImuConverter()
= default;

void ImuConverter::reset()
{

}

void ImuConverter::registerCallback(const message_actions::MessageAction action, Callback_t cb)
{
  callbacks_[action] = cb;
}

void ImuConverter::callAll(const std::vector<message_actions::MessageAction> & actions)
{
  // Get inertial data
  std::vector<float> memData;
  try {
    qi::AnyValue anyvalues = p_memory_.call<qi::AnyValue>("getListData", data_names_list_);
    tools::fromAnyValueToFloatVector(anyvalues, memData);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      naoqi::helpers::Node::get_logger(), "Exception caught in ImuConverter: " << e.what());
    return;
  }
  // angle (X,Y,Z) = memData(1,2,3);
  // gyro  (X,Y,Z) = memData(4,5,6);
  // acc   (X,Y,Z) = memData(7,8,9);

  if (memData.size() < 10) {
    throw std::runtime_error(
            "ImuConverter: expected 10 ALMemory values, got " +
            std::to_string(memData.size()));
  }

  const rclcpp::Time & stamp = helpers::Time::now();
  msg_imu_.header.stamp = stamp;

  tf2::Quaternion tf_quat;
  tf_quat.setRPY(memData[1], memData[2], memData[3]);
  msg_imu_.orientation = tf2::toMsg(tf_quat);

  msg_imu_.angular_velocity.x = memData[4];
  msg_imu_.angular_velocity.y = memData[5];
  msg_imu_.angular_velocity.z = memData[6];

  msg_imu_.linear_acceleration.x = memData[7];
  msg_imu_.linear_acceleration.y = memData[8];
  msg_imu_.linear_acceleration.z = memData[9];

  // Covariances unknown
  msg_imu_.orientation_covariance[0] = -1;
  msg_imu_.angular_velocity_covariance[0] = -1;
  msg_imu_.linear_acceleration_covariance[0] = -1;

  for (message_actions::MessageAction action: actions) {
    callbacks_[action](msg_imu_);
  }
}

}  // namespace converter

}  // namespace naoqi
