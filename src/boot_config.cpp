/*
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

#include <naoqi_driver/boot_config.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>

namespace naoqi
{

namespace
{

/** Calls visit(key, field) for every setting; keys are paths in boot_config.json. */
template<typename Visitor>
void visitFields(BootConfig & c, Visitor && visit)
{
  const auto toggle = [&visit](const std::string & key, BootConfig::Toggle & t) {
      visit(key + ".enabled", t.enabled);
    };
  const auto periodic = [&visit](const std::string & key, BootConfig::Periodic & p) {
      visit(key + ".enabled", p.enabled);
      visit(key + ".frequency", p.frequency);
    };
  const auto camera = [&visit](const std::string & key, BootConfig::Camera & cam) {
      visit(key + ".enabled", cam.enabled);
      visit(key + ".resolution", cam.resolution);
      visit(key + ".fps", cam.fps);
      visit(key + ".recorder_fps", cam.recorder_fps);
    };

  toggle("converters.info", c.info);
  toggle("converters.audio", c.audio);
  periodic("converters.logs", c.logs);
  periodic("converters.diag", c.diag);
  periodic("converters.imu_torso", c.imu_torso);
  periodic("converters.imu_base", c.imu_base);
  camera("converters.front_camera", c.front_camera);
  camera("converters.bottom_camera", c.bottom_camera);
  visit("converters.depth_camera.enabled", c.depth_camera.enabled);
  visit("converters.depth_camera.xtion_resolution", c.depth_camera.xtion_resolution);
  visit("converters.depth_camera.stereo_resolution", c.depth_camera.stereo_resolution);
  visit("converters.depth_camera.fps", c.depth_camera.fps);
  visit("converters.depth_camera.recorder_fps", c.depth_camera.recorder_fps);
  camera("converters.stereo_camera", c.stereo_camera);
  camera("converters.ir_camera", c.ir_camera);
  periodic("converters.joint_states", c.joint_states);
  visit("converters.laser.enabled", c.laser.enabled);
  visit("converters.laser.frequency", c.laser.frequency);
  visit("converters.laser.range_min", c.laser.range_min);
  visit("converters.laser.range_max", c.laser.range_max);
  periodic("converters.sonar", c.sonar);
  periodic("converters.odom", c.odom);
  toggle("converters.bumper", c.bumper);
  toggle("converters.touch_hand", c.touch_hand);
  toggle("converters.touch_head", c.touch_head);
}

/** Stores value in out if its type fits the field; false (out unchanged) otherwise. */
bool convert(const rclcpp::ParameterValue & value, bool & out)
{
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    return false;
  }
  out = value.get<bool>();
  return true;
}

bool convert(const rclcpp::ParameterValue & value, size_t & out)
{
  double number;
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    number = static_cast<double>(value.get<int64_t>());
  } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    number = std::round(value.get<double>());
  } else {
    return false;
  }
  if (number < 0) {
    return false;
  }
  out = static_cast<size_t>(number);
  return true;
}

bool convert(const rclcpp::ParameterValue & value, float & out)
{
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    out = static_cast<float>(value.get<double>());
  } else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    out = static_cast<float>(value.get<int64_t>());
  } else {
    return false;
  }
  return true;
}

}  // namespace

BootConfig parseBootConfig(const boost::property_tree::ptree & tree)
{
  BootConfig config;
  visitFields(
    config, [&tree](const std::string & key, auto & field) {
      using Field = std::decay_t<decltype(field)>;
      if constexpr (std::is_same_v<Field, size_t>) {
        // Read as int, as the driver always did, then stored unsigned.
        field = tree.get(key, static_cast<int>(field));
      } else {
        field = tree.get(key, field);
      }
    });
  return config;
}

BootConfig declareBootConfigParameters(rclcpp::Node & node, BootConfig config)
{
  visitFields(
    config, [&node](const std::string & key, auto & field) {
      using Field = std::decay_t<decltype(field)>;
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = "Setting " + key + " of boot_config.json, which gives the default";
      descriptor.read_only = true;  // the converters are created once, at start-up
      descriptor.dynamic_typing = true;  // so that 10 and 10.0 both work
      rclcpp::ParameterValue default_value;
      if constexpr (std::is_same_v<Field, size_t>) {
        default_value = rclcpp::ParameterValue(static_cast<int64_t>(field));
      } else if constexpr (std::is_same_v<Field, float>) {
        default_value = rclcpp::ParameterValue(static_cast<double>(field));
      } else {
        default_value = rclcpp::ParameterValue(field);
      }
      const rclcpp::ParameterValue value =
      node.has_parameter(key) ? node.get_parameter(key).get_parameter_value() :
      node.declare_parameter(key, default_value, descriptor);
      if (!convert(value, field)) {
        RCLCPP_WARN(
          node.get_logger(), "Parameter %s has an invalid value, using %s", key.c_str(),
          rclcpp::to_string(default_value).c_str());
      }
    });
  return config;
}

}  // namespace naoqi
