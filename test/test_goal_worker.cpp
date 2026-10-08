// Action goals run on GoalWorker threads that are joined at shutdown, never detached.

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "actions/goal_worker.hpp"
#include "helpers/connection_helpers.hpp"

using naoqi::action::ActionHandle;
using naoqi::action::GoalWorker;
using namespace std::chrono_literals;

TEST(GoalWorker, RunsWorkAndJoins)
{
  GoalWorker worker;
  std::atomic<int> runs{0};
  worker.start([&runs] {++runs;});
  worker.start([&runs] {++runs;});
  worker.join();
  EXPECT_EQ(runs.load(), 2);
}

TEST(GoalWorker, ShutdownStopsAPollingGoalAndWaitsForIt)
{
  GoalWorker worker;
  std::atomic<bool> started{false};
  std::atomic<bool> finished{false};
  worker.start(
    [&] {
      started = true;
      while (!worker.stopping()) {  // like the LED fade or audio polling loops
        std::this_thread::sleep_for(5ms);
      }
      std::this_thread::sleep_for(50ms);  // some cleanup after noticing
      finished = true;
    });
  while (!started) {
    std::this_thread::sleep_for(1ms);
  }
  worker.shutdown();
  EXPECT_TRUE(finished.load()) << "shutdown() must not return before the goal thread ends";
}

TEST(GoalWorker, FinishedThreadsAreReapedOnTheNextStart)
{
  GoalWorker worker;
  for (int i = 0; i < 50; ++i) {
    worker.start([] {});
    std::this_thread::sleep_for(1ms);
  }
  worker.shutdown();  // would hang or crash if a thread were left unjoinable
  SUCCEED();
}

TEST(ActionHandle, RunsTheHookBeforeReleasingTheServers)
{
  std::vector<std::string> events;
  {
    auto servers = std::shared_ptr<int>(
      new int(0), [&events](int * p) {
        events.push_back("servers released");
        delete p;
      });
    ActionHandle handle(servers, [&events] {events.push_back("goals stopped");});
    servers.reset();  // the handle is now the only owner
  }
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0], "goals stopped");
  EXPECT_EQ(events[1], "servers released");
}

TEST(ReconnectDelay, GrowsExponentiallyUpTo30Seconds)
{
  using naoqi::helpers::reconnectDelay;
  EXPECT_EQ(reconnectDelay(1), 1s);
  EXPECT_EQ(reconnectDelay(2), 2s);
  EXPECT_EQ(reconnectDelay(3), 4s);
  EXPECT_EQ(reconnectDelay(5), 16s);
  EXPECT_EQ(reconnectDelay(6), 30s);
  EXPECT_EQ(reconnectDelay(100), 30s);
  EXPECT_EQ(reconnectDelay(0), 1s);
}
