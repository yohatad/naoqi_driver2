// GlobalRecorder fixes: atomic started flag, no crash when HOME is unset.

#include <atomic>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <naoqi_driver/recorder/globalrecorder.hpp>

#include "test_utils.hpp"

using naoqi::recorder::GlobalRecorder;

class RecorderStateTest : public naoqi_test::RosTest {};

TEST_F(RecorderStateTest, StopRecordWorksWithoutHome)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  const char * old_home = std::getenv("HOME");
  const std::string saved = old_home ? old_home : "";
  ::unsetenv("HOME");

  GlobalRecorder global("naoqi_driver");
  global.startRecord("nohome");
  std::string bag;
  EXPECT_NO_THROW(bag = global.stopRecord());
  EXPECT_NE(bag.find("nohome_"), std::string::npos);

  if (old_home) {
    ::setenv("HOME", saved.c_str(), 1);
  }
}

TEST_F(RecorderStateTest, IsStartedCanBePolledWhileStartingAndStopping)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  GlobalRecorder global("naoqi_driver");

  std::atomic<bool> stop{false};
  std::atomic<bool> polling{false};
  std::atomic<long> polls{0};
  std::thread poller([&] {
      while (!stop) {
        (void)global.isStarted();
        ++polls;
        polling = true;
      }
    });
  // Only start toggling once the poller is running, so the two really overlap.
  while (!polling) {
    std::this_thread::yield();
  }
  for (int i = 0; i < 30; ++i) {
    global.startRecord("poll");
    global.stopRecord();
  }
  stop = true;
  poller.join();

  EXPECT_FALSE(global.isStarted());
  EXPECT_GT(polls.load(), 0);
}
