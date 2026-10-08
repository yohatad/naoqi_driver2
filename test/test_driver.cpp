// Driver-level checks that need no robot: node ownership (Driver is now created
// with std::make_shared) and the std::filesystem-based bag file management.

#include <algorithm>
#include <filesystem>
#include <memory>
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

TEST_F(DriverTest, GetFilesListReturnsBagNamesUnderWorkingDirectory)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  dir.writeFile("first.bag", 3);
  dir.writeFile("notes.txt", 3);
  dir.writeFile("nested/second.bag", 3);

  auto driver = std::make_shared<naoqi::Driver>();
  auto files = driver->getFilesList();
  std::sort(files.begin(), files.end());

  const std::vector<std::string> expected = {"first.bag", "second.bag"};
  EXPECT_EQ(files, expected);
}

TEST_F(DriverTest, RemoveAllFilesDeletesTopLevelBagsOnly)
{
  // Same as before the migration: getFiles() yields bare filenames, so only bags
  // directly in the working directory resolve and get removed.
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  dir.writeFile("first.bag", 3);
  dir.writeFile("notes.txt", 3);
  dir.writeFile("nested/second.bag", 3);

  auto driver = std::make_shared<naoqi::Driver>();
  driver->removeAllFiles();

  EXPECT_FALSE(std::filesystem::exists(dir.path() / "first.bag"));
  EXPECT_TRUE(std::filesystem::exists(dir.path() / "notes.txt"));
  EXPECT_TRUE(std::filesystem::exists(dir.path() / "nested/second.bag"));
}

TEST_F(DriverTest, MinidumpRefusesWhenLoggingIsDisabled)
{
  auto driver = std::make_shared<naoqi::Driver>();
  EXPECT_EQ(
    driver->minidump("unit"),
    "Log is not enabled, please enable logging before calling minidump");
}
