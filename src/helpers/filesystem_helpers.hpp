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

#include <iostream>
#include <qi/session.hpp>

#include <filesystem>

#include "string_helpers.hpp"

#ifdef AMENT_BUILD
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace naoqi
{
namespace helpers
{
namespace filesystem
{

static const long folderMaximumSize = 2000000000;

inline void getFoldersize(std::string rootFolder, long & file_size)
{
  helpers::string::replaceAll(rootFolder, "\\\\", "\\");
  std::filesystem::path folderPath(rootFolder);
  if (std::filesystem::exists(folderPath)) {
    std::filesystem::directory_iterator end_itr;

    for (std::filesystem::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte) {
      std::filesystem::path filePath(dirIte->path());
      try {
        if (!std::filesystem::is_directory(dirIte->status()) ) {
          file_size = file_size + std::filesystem::file_size(filePath);
        } else {
          getFoldersize(filePath.string(), file_size);
        }
      } catch (std::exception & e) {
        std::cout << e.what() << std::endl;
      }
    }
  }
}

inline void getFiles(
  const std::filesystem::path & root, const std::string & ext,
  std::vector<std::filesystem::path> & ret)
{
  if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) {return;}

  std::filesystem::recursive_directory_iterator it(root);
  std::filesystem::recursive_directory_iterator endit;

  while (it != endit) {
    if (std::filesystem::is_regular_file(*it) && it->path().extension() == ext) {
      ret.push_back(it->path().filename());
    }
    ++it;
  }
}

inline void getFilesSize(const std::filesystem::path & root, long & file_size)
{
  std::vector<std::filesystem::path> files_path;
  getFiles(root, ".bag", files_path);
  for (std::vector<std::filesystem::path>::const_iterator it = files_path.begin();
    it != files_path.end(); it++)
  {
    try {
      file_size = file_size + std::filesystem::file_size(*it);
    } catch (std::exception & e) {
      std::cout << e.what() << std::endl;
    }
  }
}

/** Boot config loader */
static const std::string boot_config_file_name = "boot_config.json";
inline std::string & getBootConfigFile()
{
#ifdef AMENT_BUILD
  static std::string path = ament_index_cpp::get_package_share_directory("naoqi_driver") +
    "/share/" + boot_config_file_name;
  std::cout << "found an ament prefix " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData("/", boot_config_file_name);
  std::cout << "found a qibuild path " << path << std::endl;
  return path;
#endif
}

/* URDF loader */
inline std::string & getURDF(std::string filename)
{
#ifdef AMENT_BUILD
  static std::string path = ament_index_cpp::get_package_share_directory("naoqi_driver") +
    "/share/urdf/" + filename;
  std::cout << "found a ament URDF " << path << std::endl;
  return path;
#else
  static std::string path = qi::path::findData("/urdf/", filename);
  std::cout << "found a qibuild URDF " << path << std::endl;
  return path;
#endif
}

} // filesystem
} //helpers
} // naoqi

#endif
