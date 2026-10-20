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

#ifndef NAOQI_DRIVER__BOOT_CONFIG_HPP_
#define NAOQI_DRIVER__BOOT_CONFIG_HPP_

#include <cstddef>

#include <boost/property_tree/ptree.hpp>
#include <rclcpp/rclcpp.hpp>

namespace naoqi
{

/**
 * @brief Typed view of config/boot_config.json ("converters" section), where every
 * setting can be overridden by a parameter (see declareBootConfigParameters()).
 * Every field keeps the default below when its key is missing. Camera resolutions
 * use the NAOqi codes from the file's legend: QQVGA = 0, QVGA = 1, VGA = 2.
 */
struct BootConfig
{
  struct Toggle
  {
    bool enabled;
  };

  struct Periodic
  {
    bool enabled;
    size_t frequency;  // Hz
  };

  struct Camera
  {
    bool enabled;
    size_t resolution;
    size_t fps;
    size_t recorder_fps;
  };

  struct DepthCamera
  {
    bool enabled;
    size_t xtion_resolution;
    size_t stereo_resolution;
    size_t fps;
    size_t recorder_fps;
  };

  struct Laser
  {
    bool enabled;
    size_t frequency;  // Hz
    float range_min;  // m
    float range_max;  // m
  };

  Toggle info{true};  // published once per new subscriber, so it has no frequency
  Toggle audio{true};  // pushed by NAOqi as buffers arrive, so it has no frequency
  Periodic logs{true, 10};
  Periodic diag{true, 10};
  Periodic imu_torso{true, 10};
  Periodic imu_base{true, 10};
  Camera front_camera{true, 1, 10, 5};
  Camera bottom_camera{true, 1, 10, 5};
  DepthCamera depth_camera{true, 1, 9, 10, 5};  // stereo_resolution 9 = Q720p
  Camera stereo_camera{true, 15, 10, 5};  // resolution 15 = QQ720px2
  Camera ir_camera{true, 1, 10, 5};
  Periodic joint_states{true, 50};
  Laser laser{true, 10, 0.1f, 3.0f};
  Periodic sonar{true, 10};
  Periodic odom{true, 10};
  Toggle bumper{true};
  Toggle touch_hand{true};
  Toggle touch_head{true};
};

/** Reads the "converters" section of a parsed boot_config.json. */
BootConfig parseBootConfig(const boost::property_tree::ptree & tree);

/**
 * Declares every setting as a read-only parameter named like its JSON path
 * (converters.front_camera.enabled, ...), with the value in config as default.
 * @return config with the values the node was given; an invalid one keeps the default
 */
BootConfig declareBootConfigParameters(rclcpp::Node & node, BootConfig config);

}  // namespace naoqi

#endif  // NAOQI_DRIVER__BOOT_CONFIG_HPP_
