// BootConfig replaces the string-keyed property_tree lookups that used to be spread
// through Driver::registerDefaultConverter. The defaults must match the old ones.

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

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

namespace
{

/** A node given these parameters, as from the command line or a launch file. */
std::shared_ptr<rclcpp::Node> nodeWith(const std::vector<rclcpp::Parameter> & overrides)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  static int count = 0;
  return std::make_shared<rclcpp::Node>(
    "boot_config_test_" + std::to_string(count++),
    rclcpp::NodeOptions().parameter_overrides(overrides));
}

}  // namespace

TEST(BootConfigParameters, FileValuesBecomeTheDefaults)
{
  auto node = nodeWith({});
  naoqi::BootConfig file;
  file.laser.frequency = 7;
  file.front_camera.enabled = false;
  const auto config = naoqi::declareBootConfigParameters(*node, file);

  EXPECT_EQ(config.laser.frequency, 7u);
  EXPECT_FALSE(config.front_camera.enabled);
  EXPECT_EQ(node->get_parameter("converters.laser.frequency").as_int(), 7);
  EXPECT_FALSE(node->get_parameter("converters.front_camera.enabled").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("converters.laser.range_max").as_double(), 3.0);
}

TEST(BootConfigParameters, GivenParametersOverrideTheFile)
{
  auto node = nodeWith(
  {
    rclcpp::Parameter("converters.front_camera.enabled", true),
    rclcpp::Parameter("converters.front_camera.resolution", 2),
    rclcpp::Parameter("converters.imu_base.frequency", 12.0),  // a double for a count
    rclcpp::Parameter("converters.laser.range_max", 5),  // an int for a float
  });
  naoqi::BootConfig file;
  file.front_camera.enabled = false;
  const auto config = naoqi::declareBootConfigParameters(*node, file);

  EXPECT_TRUE(config.front_camera.enabled);
  EXPECT_EQ(config.front_camera.resolution, 2u);
  EXPECT_EQ(config.imu_base.frequency, 12u);
  EXPECT_FLOAT_EQ(config.laser.range_max, 5.f);
  EXPECT_EQ(config.bottom_camera.fps, file.bottom_camera.fps);  // untouched
}

TEST(BootConfigParameters, InvalidValuesKeepTheFileValue)
{
  auto node = nodeWith(
  {
    rclcpp::Parameter("converters.odom.frequency", -5),
    rclcpp::Parameter("converters.sonar.enabled", "yes"),
  });
  naoqi::BootConfig file;
  const auto config = naoqi::declareBootConfigParameters(*node, file);
  EXPECT_EQ(config.odom.frequency, file.odom.frequency);
  EXPECT_EQ(config.sonar.enabled, file.sonar.enabled);
}

TEST(BootConfigParameters, AreReadOnlyOnceDeclared)
{
  // Converters are built once: a later change would silently do nothing.
  auto node = nodeWith({});
  naoqi::declareBootConfigParameters(*node, naoqi::BootConfig());
  const auto result = node->set_parameter(rclcpp::Parameter("converters.odom.enabled", false));
  EXPECT_FALSE(result.successful);
}
