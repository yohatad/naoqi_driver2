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

#ifndef FILESYSTEM_HELPERS_HPP
#define FILESYSTEM_HELPERS_HPP

#include <naoqi_driver/ros_helpers.hpp>
#include <qi/session.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <vector>

#ifdef AMENT_BUILD
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace naoqi
{
namespace helpers
{
namespace filesystem
{

/** Above this total size of bags in the working directory, minidump refuses (2 GB). */
static const std::uintmax_t folderMaximumSize = 2000000000;

/** Total size of the files under path, recursively; 0 if it does not exist. */
inline std::uintmax_t getSize(const std::filesystem::path & path)
{
  std::error_code ec;
  if (std::filesystem::is_regular_file(path, ec)) {
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    return ec ? 0 : size;
  }
  std::uintmax_t total = 0;
  for (std::filesystem::recursive_directory_iterator it(path, ec), end; !ec && it != end;
    it.increment(ec))
  {
    if (it->is_regular_file(ec)) {
      const std::uintmax_t size = it->file_size(ec);
      total += ec ? 0 : size;
      ec.clear();
    }
  }
  return total;
}

/** The rosbag2 bags directly in folder: subfolders with a metadata.yaml or a .db3 file. */
inline std::vector<std::filesystem::path> getBags(const std::filesystem::path & folder)
{
  std::vector<std::filesystem::path> bags;
  std::error_code ec;
  for (std::filesystem::directory_iterator it(folder, ec), end; !ec && it != end;
    it.increment(ec))
  {
    if (!it->is_directory(ec)) {
      continue;
    }
    bool is_bag = std::filesystem::exists(it->path() / "metadata.yaml", ec);
    std::error_code ec_files;
    for (std::filesystem::directory_iterator file(it->path(), ec_files), files_end;
      !is_bag && !ec_files && file != files_end; file.increment(ec_files))
    {
      is_bag = file->path().extension() == ".db3";
    }
    if (is_bag) {
      bags.push_back(it->path());
    }
  }
  std::sort(bags.begin(), bags.end());
  return bags;
}

/** Total size of the bags directly in folder. */
inline std::uintmax_t getBagsSize(const std::filesystem::path & folder)
{
  std::uintmax_t total = 0;
  for (const auto & bag : getBags(folder)) {
    total += getSize(bag);
  }
  return total;
}

static const std::string boot_config_file_name = "boot_config.json";
inline std::string & getBootConfigFile()
{
#ifdef AMENT_BUILD
  static std::string path = ament_index_cpp::get_package_share_directory("naoqi_driver") +
    "/config/" + boot_config_file_name;
  RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), "found an ament prefix " << path);
  return path;
#else
  static std::string path = qi::path::findData("/", boot_config_file_name);
  RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), "found a qibuild path " << path);
  return path;
#endif
}

/* URDF loader */
inline std::string & getURDF(std::string filename)
{
#ifdef AMENT_BUILD
  static std::string path = ament_index_cpp::get_package_share_directory("naoqi_driver") +
    "/urdf/" + filename;
  RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), "found a ament URDF " << path);
  return path;
#else
  static std::string path = qi::path::findData("/urdf/", filename);
  RCLCPP_DEBUG_STREAM(naoqi::helpers::Node::get_logger(), "found a qibuild URDF " << path);
  return path;
#endif
}

}  // namespace filesystem
}  // namespace helpers
}  // namespace naoqi

#endif
