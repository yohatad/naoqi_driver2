#ifndef NAOQI_DRIVER_TEST_UTILS_HPP
#define NAOQI_DRIVER_TEST_UTILS_HPP

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

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

/**
 * @brief Captures every rcutils/rclcpp log message while alive, to check that a
 * warning was emitted. Only one instance may exist at a time.
 */
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    current() = this;
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }

  ~LogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
    current() = nullptr;
  }

  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;

  bool contains(const std::string & text) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & m : messages_) {
      if (m.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  bool waitFor(
    const std::string & text,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (contains(text)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return contains(text);
  }

private:
  static LogCapture * & current()
  {
    static LogCapture * instance = nullptr;
    return instance;
  }

  static void handler(
    const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
    const char * format, va_list * args)
  {
    char buffer[2048];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    if (LogCapture * self = current()) {
      std::lock_guard<std::mutex> lock(self->mutex_);
      self->messages_.emplace_back(buffer);
    }
  }

  rcutils_logging_output_handler_t previous_;
  mutable std::mutex mutex_;
  std::vector<std::string> messages_;
};

}  // namespace naoqi_test

#endif
