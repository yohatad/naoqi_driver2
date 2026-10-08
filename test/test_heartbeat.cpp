// Heartbeat against the in-process fake NAOqi: it counts in ALMemory only while the
// node's callbacks run, marks a clean exit, and warns once per outage.

#include <chrono>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "fake_naoqi.hpp"
#include "heartbeat.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using naoqi::Heartbeat;

class HeartbeatTest : public naoqi_test::RosTest
{
protected:
  size_t beats() const
  {
    return fake_.log.count(std::string("insertData(") + Heartbeat::kKey + ")");
  }

  int lastBeat() const {return static_cast<int>(fake_.memory->inserted(Heartbeat::kKey).toInt());}

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      std::this_thread::sleep_for(5ms);
    }
  }

  bool spinUntilBeats(size_t n)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (beats() < n && std::chrono::steady_clock::now() < end) {
      spinFor(10ms);
    }
    return beats() >= n;
  }

  naoqi_test::FakeNaoqi fake_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(HeartbeatTest, CountsWhileCallbacksRun)
{
  Heartbeat heartbeat(node_.get(), fake_.session, 20ms);
  executor_.add_node(node_);
  ASSERT_TRUE(spinUntilBeats(3));
  EXPECT_GE(lastBeat(), 3);
}

TEST_F(HeartbeatTest, GoesStaleWhenCallbacksStop)
{
  // A hung callback thread cannot process /cmd_vel either: the watchdog must notice.
  Heartbeat heartbeat(node_.get(), fake_.session, 20ms);
  executor_.add_node(node_);
  ASSERT_TRUE(spinUntilBeats(2));
  executor_.remove_node(node_);
  std::this_thread::sleep_for(100ms);
  const size_t stalled = beats();
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(beats(), stalled);
}

TEST_F(HeartbeatTest, MarksACleanExit)
{
  {
    Heartbeat heartbeat(node_.get(), fake_.session, 20ms);
    executor_.add_node(node_);
    ASSERT_TRUE(spinUntilBeats(2));
    executor_.remove_node(node_);
  }
  EXPECT_EQ(lastBeat(), Heartbeat::kStopped);
}

TEST_F(HeartbeatTest, WarnsOncePerOutage)
{
  naoqi_test::LogCapture logs;
  const std::string warning = "Writing the heartbeat to ALMemory failed";
  Heartbeat heartbeat(node_.get(), fake_.session, 20ms);
  executor_.add_node(node_);

  fake_.memory->fail_inserts = true;
  spinFor(300ms);
  EXPECT_EQ(logs.count(warning), 1u);

  fake_.memory->fail_inserts = false;
  const size_t before = beats();
  ASSERT_TRUE(spinUntilBeats(before + 2));
  EXPECT_EQ(logs.count(warning), 1u);

  fake_.memory->fail_inserts = true;
  spinFor(300ms);
  EXPECT_EQ(logs.count(warning), 2u);
  fake_.memory->fail_inserts = false;
}
