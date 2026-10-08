// BootConfig replaces the string-keyed property_tree lookups that used to be spread
// through Driver::registerDefaultConverter. The defaults must match the old ones.

#include <sstream>
#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <gtest/gtest.h>

#include <naoqi_driver/boot_config.hpp>

namespace
{

naoqi::BootConfig parse(const std::string & json)
{
  std::istringstream in(json);
  boost::property_tree::ptree tree;
  boost::property_tree::read_json(in, tree);
  return naoqi::parseBootConfig(tree);
}

}  // namespace

TEST(BootConfig, EmptyFileKeepsTheDriverDefaults)
{
  const auto c = naoqi::parseBootConfig(boost::property_tree::ptree());

  EXPECT_TRUE(c.info.enabled);
  EXPECT_TRUE(c.audio.enabled);
  EXPECT_EQ(c.logs.frequency, 10u);
  EXPECT_EQ(c.diag.frequency, 10u);
  EXPECT_EQ(c.imu_torso.frequency, 10u);
  EXPECT_EQ(c.imu_base.frequency, 10u);
  EXPECT_EQ(c.front_camera.resolution, 1u);
  EXPECT_EQ(c.front_camera.fps, 10u);
  EXPECT_EQ(c.front_camera.recorder_fps, 5u);
  EXPECT_EQ(c.bottom_camera.resolution, 1u);
  EXPECT_EQ(c.depth_camera.xtion_resolution, 1u);
  EXPECT_EQ(c.depth_camera.stereo_resolution, 9u);
  EXPECT_EQ(c.stereo_camera.resolution, 15u);
  EXPECT_EQ(c.ir_camera.resolution, 1u);
  EXPECT_EQ(c.joint_states.frequency, 50u);
  EXPECT_EQ(c.laser.frequency, 10u);
  EXPECT_FLOAT_EQ(c.laser.range_min, 0.1f);
  EXPECT_FLOAT_EQ(c.laser.range_max, 3.0f);
  EXPECT_EQ(c.sonar.frequency, 10u);
  EXPECT_EQ(c.odom.frequency, 10u);
  EXPECT_TRUE(c.bumper.enabled);
  EXPECT_TRUE(c.touch_hand.enabled);
  EXPECT_TRUE(c.touch_head.enabled);
}

TEST(BootConfig, ReadsEveryKindOfEntry)
{
  const auto c = parse(
    R"({ "converters": {
    "audio": { "enabled": false },
    "imu_base": { "enabled": true, "frequency": 100 },
    "front_camera": { "enabled": false, "resolution": 2, "fps": 15, "recorder_fps": 3 },
    "depth_camera": { "enabled": true, "xtion_resolution": 0, "stereo_resolution": 10,
                      "fps": 7, "recorder_fps": 2 },
    "laser": { "enabled": false, "frequency": 5, "range_min": 0.25, "range_max": 4.5 },
    "bumper": { "enabled": false }
  } })");

  EXPECT_FALSE(c.audio.enabled);
  EXPECT_TRUE(c.imu_base.enabled);
  EXPECT_EQ(c.imu_base.frequency, 100u);
  EXPECT_FALSE(c.front_camera.enabled);
  EXPECT_EQ(c.front_camera.resolution, 2u);
  EXPECT_EQ(c.front_camera.fps, 15u);
  EXPECT_EQ(c.front_camera.recorder_fps, 3u);
  EXPECT_EQ(c.depth_camera.xtion_resolution, 0u);
  EXPECT_EQ(c.depth_camera.stereo_resolution, 10u);
  EXPECT_EQ(c.depth_camera.fps, 7u);
  EXPECT_EQ(c.depth_camera.recorder_fps, 2u);
  EXPECT_FALSE(c.laser.enabled);
  EXPECT_EQ(c.laser.frequency, 5u);
  EXPECT_FLOAT_EQ(c.laser.range_min, 0.25f);
  EXPECT_FLOAT_EQ(c.laser.range_max, 4.5f);
  EXPECT_FALSE(c.bumper.enabled);
}

TEST(BootConfig, MissingKeysFallBackFieldByField)
{
  const auto c = parse(R"({ "converters": { "front_camera": { "fps": 30 } } })");
  EXPECT_EQ(c.front_camera.fps, 30u);
  EXPECT_TRUE(c.front_camera.enabled);
  EXPECT_EQ(c.front_camera.resolution, 1u);
  EXPECT_EQ(c.front_camera.recorder_fps, 5u);
}

TEST(BootConfig, ShippedConfigFilesParse)
{
  for (const std::string file : {"/share/boot_config.json", "/test/robot/boot_config_full.json"}) {
    boost::property_tree::ptree tree;
    ASSERT_NO_THROW(boost::property_tree::read_json(NAOQI_DRIVER_SOURCE_DIR + file, tree)) << file;
    EXPECT_NO_THROW(naoqi::parseBootConfig(tree)) << file;
  }

  boost::property_tree::ptree tree;
  boost::property_tree::read_json(
    std::string(NAOQI_DRIVER_SOURCE_DIR) + "/test/robot/boot_config_full.json", tree);
  const auto full = naoqi::parseBootConfig(tree);
  EXPECT_TRUE(full.front_camera.enabled);
  EXPECT_TRUE(full.audio.enabled);
  EXPECT_FALSE(full.stereo_camera.enabled);
}
