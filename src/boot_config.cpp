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

#include <string>

namespace naoqi
{

namespace
{

// Integer settings are read as int, as the driver always did, then stored unsigned.
size_t getCount(const boost::property_tree::ptree & tree, const std::string & key, size_t fallback)
{
  return tree.get(key, static_cast<int>(fallback));
}

void read(const boost::property_tree::ptree & tree, const std::string & key, BootConfig::Toggle & t)
{
  t.enabled = tree.get(key + ".enabled", t.enabled);
}

void read(
  const boost::property_tree::ptree & tree, const std::string & key,
  BootConfig::Periodic & p)
{
  p.enabled = tree.get(key + ".enabled", p.enabled);
  p.frequency = getCount(tree, key + ".frequency", p.frequency);
}

void read(const boost::property_tree::ptree & tree, const std::string & key, BootConfig::Camera & c)
{
  c.enabled = tree.get(key + ".enabled", c.enabled);
  c.resolution = getCount(tree, key + ".resolution", c.resolution);
  c.fps = getCount(tree, key + ".fps", c.fps);
  c.recorder_fps = getCount(tree, key + ".recorder_fps", c.recorder_fps);
}

void read(
  const boost::property_tree::ptree & tree, const std::string & key,
  BootConfig::DepthCamera & c)
{
  c.enabled = tree.get(key + ".enabled", c.enabled);
  c.xtion_resolution = getCount(tree, key + ".xtion_resolution", c.xtion_resolution);
  c.stereo_resolution = getCount(tree, key + ".stereo_resolution", c.stereo_resolution);
  c.fps = getCount(tree, key + ".fps", c.fps);
  c.recorder_fps = getCount(tree, key + ".recorder_fps", c.recorder_fps);
}

void read(const boost::property_tree::ptree & tree, const std::string & key, BootConfig::Laser & l)
{
  l.enabled = tree.get(key + ".enabled", l.enabled);
  l.frequency = getCount(tree, key + ".frequency", l.frequency);
  l.range_min = tree.get<float>(key + ".range_min", l.range_min);
  l.range_max = tree.get<float>(key + ".range_max", l.range_max);
}

}  // namespace

BootConfig parseBootConfig(const boost::property_tree::ptree & tree)
{
  BootConfig config;
  read(tree, "converters.info", config.info);
  read(tree, "converters.audio", config.audio);
  read(tree, "converters.logs", config.logs);
  read(tree, "converters.diag", config.diag);
  read(tree, "converters.imu_torso", config.imu_torso);
  read(tree, "converters.imu_base", config.imu_base);
  read(tree, "converters.front_camera", config.front_camera);
  read(tree, "converters.bottom_camera", config.bottom_camera);
  read(tree, "converters.depth_camera", config.depth_camera);
  read(tree, "converters.stereo_camera", config.stereo_camera);
  read(tree, "converters.ir_camera", config.ir_camera);
  read(tree, "converters.joint_states", config.joint_states);
  read(tree, "converters.laser", config.laser);
  read(tree, "converters.sonar", config.sonar);
  read(tree, "converters.odom", config.odom);
  read(tree, "converters.bumper", config.bumper);
  read(tree, "converters.touch_hand", config.touch_hand);
  read(tree, "converters.touch_head", config.touch_head);
  return config;
}

}  // namespace naoqi
