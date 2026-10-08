#ifndef NAOQI_DRIVER_TEST_UTILS_HPP
#define NAOQI_DRIVER_TEST_UTILS_HPP

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <naoqi_driver/ros_helpers.hpp>

namespace naoqi_test
{

/**
 * @brief Fixture that brings up rclcpp and registers a node with helpers::Node,
 * which helpers::Time::now() and the recorders rely on.
 */
class RosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("naoqi_driver_test");
    naoqi::helpers::Node::setNode(node_);
  }

  static void TearDownTestSuite()
  {
    naoqi::helpers::Node::setNode(nullptr);
    node_.reset();
    rclcpp::shutdown();
  }

  inline static std::shared_ptr<rclcpp::Node> node_;
};

/**
 * @brief Creates a unique empty directory, optionally makes it the working
 * directory, and removes it (restoring the previous working directory) on scope exit.
 */
class TempDir
{
public:
  explicit TempDir(bool chdir_into = false)
  : previous_cwd_(std::filesystem::current_path()), chdir_into_(chdir_into)
  {
    static std::atomic<int> counter{0};
    path_ = std::filesystem::temp_directory_path() /
      ("naoqi_driver_test_" + std::to_string(::getpid()) + "_" + std::to_string(counter++));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_);
    if (chdir_into_) {
      std::filesystem::current_path(path_);
    }
  }

  ~TempDir()
  {
    if (chdir_into_) {
      std::filesystem::current_path(previous_cwd_);
    }
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;

  const std::filesystem::path & path() const {return path_;}

  /** Writes a file of exactly size bytes at path_/relative, creating parent folders. */
  std::filesystem::path writeFile(const std::string & relative, std::size_t size) const
  {
    const auto file = path_ / relative;
    std::filesystem::create_directories(file.parent_path());
    std::ofstream out(file, std::ios::binary);
    out << std::string(size, 'x');
    return file;
  }

private:
  std::filesystem::path path_;
  std::filesystem::path previous_cwd_;
  bool chdir_into_;
};

} // naoqi_test

#endif
