// Driver-level checks that need no robot: node ownership (Driver is now created
// with std::make_shared), converter registration and the minidump guard.

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <gtest/gtest.h>

#include <naoqi_driver/naoqi_driver.hpp>
#include <naoqi_driver/ros_helpers.hpp>

#include "test_utils.hpp"

class DriverTest : public naoqi_test::RosTest {};

TEST_F(DriverTest, BoostOwnedNodeCannotUseSharedFromThis)
{
  // Why the Driver moved off boost::make_shared: rclcpp::Node derives from
  // std::enable_shared_from_this, which a boost::shared_ptr never initialises.
  auto node = boost::make_shared<rclcpp::Node>("boost_owned_node");
  EXPECT_THROW(node->shared_from_this(), std::bad_weak_ptr);
}

TEST_F(DriverTest, StdOwnedDriverSupportsSharedFromThis)
{
  auto driver = std::make_shared<naoqi::Driver>();
  naoqi::helpers::Node::setNode(driver);

  std::shared_ptr<rclcpp::Node> self;
  ASSERT_NO_THROW(self = driver->shared_from_this());
  EXPECT_EQ(self.get(), driver.get());
  EXPECT_EQ(naoqi::helpers::Node::get_node().get(), driver.get());
  EXPECT_EQ(naoqi::helpers::Node::get_logger().get_name(), std::string("naoqi_driver"));

  naoqi::helpers::Node::setNode(node_);
}

TEST_F(DriverTest, MinidumpRefusesWhenLoggingIsDisabled)
{
  auto driver = std::make_shared<naoqi::Driver>();
  try {
    driver->minidump("unit");
    FAIL() << "minidump must refuse while logging is off";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("minidump_duration"), std::string::npos) << e.what();
  }
}

namespace
{

/** Minimal converter, registered through the converter::Converter interface. */
struct CountingConverter : naoqi::converter::Converter
{
  std::string name() const override {return "counting";}
  float frequency() const override {return 0.f;}
  void reset() override {++resets;}
  void callAll(const std::vector<naoqi::message_actions::MessageAction> &) override {++calls;}
  int resets = 0;
  int calls = 0;
};

}  // namespace

TEST_F(DriverTest, RegisteredConvertersAreResetAndListed)
{
  auto driver = std::make_shared<naoqi::Driver>();
  auto converter = std::make_shared<CountingConverter>();
  driver->registerConverter(converter);

  EXPECT_EQ(converter->resets, 1);
  const auto names = driver->getAvailableConverters();
  EXPECT_NE(std::find(names.begin(), names.end(), "counting"), names.end());
}
