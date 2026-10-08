// helpers/filesystem_helpers.hpp moved from boost::filesystem to std::filesystem.
// The pre-migration implementation is kept below (namespace legacy, verbatim apart
// from the namespace) so each helper is checked against what the driver did before.

#include <algorithm>
#include <string>
#include <vector>

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include "helpers/filesystem_helpers.hpp"
#include "test_utils.hpp"

namespace fsh = naoqi::helpers::filesystem;
using naoqi_test::TempDir;

namespace legacy
{

inline void getFoldersize(std::string rootFolder, long& file_size){
  boost::algorithm::replace_all(rootFolder, "\\\\", "\\");
  boost::filesystem::path folderPath(rootFolder);
  if (boost::filesystem::exists(folderPath)){
    boost::filesystem::directory_iterator end_itr;

    for (boost::filesystem::directory_iterator dirIte(rootFolder); dirIte != end_itr; ++dirIte )
    {
      boost::filesystem::path filePath(dirIte->path());
      try{
        if (!boost::filesystem::is_directory(dirIte->status()) )
        {
          file_size = file_size + boost::filesystem::file_size(filePath);
        }else{
          getFoldersize(filePath.string(),file_size);
        }
      }catch(std::exception& e){
        std::cout << e.what() << std::endl;
      }
    }
  }
}

inline void getFiles(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
  if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

  boost::filesystem::recursive_directory_iterator it(root);
  boost::filesystem::recursive_directory_iterator endit;

  while(it != endit)
  {
    if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext)
    {
      ret.push_back(it->path().filename());
    }
    ++it;
  }
}

inline void getFilesSize(const boost::filesystem::path& root, long& file_size)
{
  std::vector<boost::filesystem::path> files_path;
  getFiles(root, ".bag", files_path);
  for (std::vector<boost::filesystem::path>::const_iterator it=files_path.begin();
       it!=files_path.end(); it++)
  {
    try{
      file_size = file_size + boost::filesystem::file_size(*it);
    }catch(std::exception& e){
      std::cout << e.what() << std::endl;
    }
  }
}

} // namespace legacy

namespace
{

std::vector<std::string> stdGetFiles(const std::filesystem::path& root, const std::string& ext)
{
  std::vector<std::filesystem::path> files;
  fsh::getFiles(root, ext, files);
  std::vector<std::string> names;
  for (const auto& f : files)
    names.push_back(f.string());
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> legacyGetFiles(const std::filesystem::path& root, const std::string& ext)
{
  std::vector<boost::filesystem::path> files;
  legacy::getFiles(boost::filesystem::path(root.string()), ext, files);
  std::vector<std::string> names;
  for (const auto& f : files)
    names.push_back(f.string());
  std::sort(names.begin(), names.end());
  return names;
}

/** a.bag(100) b.txt(50) e.bagx(9) sub/c.bag(30) sub/deeper/d.bag(7) => 196 bytes total */
void populate(const TempDir& dir)
{
  dir.writeFile("a.bag", 100);
  dir.writeFile("b.txt", 50);
  dir.writeFile("e.bagx", 9);
  dir.writeFile("sub/c.bag", 30);
  dir.writeFile("sub/deeper/d.bag", 7);
}

} // namespace

TEST(FilesystemHelpers, GetFilesFindsBagsRecursivelyAsBareFilenames)
{
  TempDir dir;
  populate(dir);

  const std::vector<std::string> expected = {"a.bag", "c.bag", "d.bag"};
  EXPECT_EQ(stdGetFiles(dir.path(), ".bag"), expected);
  EXPECT_EQ(stdGetFiles(dir.path(), ".bag"), legacyGetFiles(dir.path(), ".bag"));
}

TEST(FilesystemHelpers, GetFilesIgnoresMissingRootAndPlainFiles)
{
  TempDir dir;
  const auto file = dir.writeFile("x.bag", 1);

  EXPECT_TRUE(stdGetFiles(dir.path() / "does_not_exist", ".bag").empty());
  EXPECT_TRUE(stdGetFiles(file, ".bag").empty());
  EXPECT_EQ(stdGetFiles(file, ".bag"), legacyGetFiles(file, ".bag"));
}

TEST(FilesystemHelpers, GetFoldersizeSumsEveryFileRecursively)
{
  TempDir dir;
  populate(dir);

  long std_size = 0;
  long legacy_size = 0;
  fsh::getFoldersize(dir.path().string(), std_size);
  legacy::getFoldersize(dir.path().string(), legacy_size);

  EXPECT_EQ(std_size, 196);
  EXPECT_EQ(std_size, legacy_size);
}

TEST(FilesystemHelpers, GetFoldersizeOnMissingFolderLeavesSizeUntouched)
{
  TempDir dir;
  long size = 5;
  fsh::getFoldersize((dir.path() / "missing").string(), size);
  EXPECT_EQ(size, 5);
}

TEST(FilesystemHelpers, GetFilesSizeMatchesLegacyFromDriverWorkingDirectory)
{
  // The driver always calls this with root == current_path(). getFiles() returns
  // bare filenames, so only top-level bags resolve and nested ones are skipped
  // (pre-existing behaviour, kept unchanged by the migration).
  TempDir dir(/*chdir_into=*/true);
  populate(dir);

  long std_size = 0;
  long legacy_size = 0;
  fsh::getFilesSize(std::filesystem::current_path(), std_size);
  legacy::getFilesSize(boost::filesystem::current_path(), legacy_size);

  EXPECT_EQ(std_size, 100);
  EXPECT_EQ(std_size, legacy_size);
}
