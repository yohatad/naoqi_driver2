// helpers/filesystem_helpers.hpp: sizes of files and rosbag2 bags (folders), which the
// minidump disk-space check relies on.

#include <filesystem>
#include <vector>

#include <gtest/gtest.h>

#include "helpers/filesystem_helpers.hpp"
#include "test_utils.hpp"

namespace fsh = naoqi::helpers::filesystem;
using naoqi_test::TempDir;

TEST(FilesystemHelpers, GetSizeSumsEveryFileRecursively)
{
  TempDir dir;
  dir.writeFile("a.txt", 100);
  dir.writeFile("sub/b.txt", 30);
  dir.writeFile("sub/deeper/c.txt", 7);

  EXPECT_EQ(fsh::getSize(dir.path()), 137u);
  EXPECT_EQ(fsh::getSize(dir.path() / "sub"), 37u);
  EXPECT_EQ(fsh::getSize(dir.path() / "a.txt"), 100u);
  EXPECT_EQ(fsh::getSize(dir.path() / "missing"), 0u);
}

TEST(FilesystemHelpers, GetBagsFindsBagFoldersOnly)
{
  TempDir dir;
  dir.writeFile("closed/metadata.yaml", 10);
  dir.writeFile("closed/closed_0.db3", 100);
  dir.writeFile("open/open_0.db3", 50);  // metadata.yaml is written on close
  dir.writeFile("notes/readme.txt", 5);
  dir.writeFile("old.bag", 20);
  dir.writeFile("nested/inner/metadata.yaml", 1);  // only bags directly in the folder count

  const std::vector<std::filesystem::path> expected = {
    dir.path() / "closed", dir.path() / "open"};
  EXPECT_EQ(fsh::getBags(dir.path()), expected);
  EXPECT_EQ(fsh::getBagsSize(dir.path()), 160u);
}

TEST(FilesystemHelpers, GetBagsOfAMissingFolderIsEmpty)
{
  TempDir dir;
  EXPECT_TRUE(fsh::getBags(dir.path() / "missing").empty());
  EXPECT_EQ(fsh::getBagsSize(dir.path() / "missing"), 0u);
}
