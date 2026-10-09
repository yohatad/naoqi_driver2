// helpers::ClockOffset: camera capture times on the robot's clock, mapped to the
// local clock with the smallest recent arrival - capture difference.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "helpers/clock_offset.hpp"

using naoqi::helpers::ClockOffset;

namespace
{

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kSecond = 1'000 * kMs;
// The robot's clock is far from the local one (e.g. time since boot).
constexpr int64_t kRobotStart = 5'000 * kSecond;
constexpr int64_t kLocalStart = 1'790'000'000 * kSecond;

}  // namespace

TEST(ClockOffset, KeepsTheSpacingOfCapturesDespiteVariableDelays)
{
  ClockOffset clock;
  // 15 fps captures arriving after 20 to 80 ms.
  const std::vector<int64_t> delays_ms = {80, 20, 55, 30, 70, 25, 60, 45};
  std::vector<int64_t> stamps;
  for (size_t i = 0; i < delays_ms.size(); ++i) {
    const int64_t capture = static_cast<int64_t>(i) * 66 * kMs;
    const int64_t arrival = kLocalStart + capture + delays_ms[i] * kMs;
    const int64_t stamp = clock.toLocal(kRobotStart + capture, arrival);
    EXPECT_LE(stamp, arrival) << "frame " << i;
    stamps.push_back(stamp);
  }
  // From the shortest delay (frame 1) on: stamp = capture + offset + 20 ms, exactly.
  for (size_t i = 1; i < stamps.size(); ++i) {
    EXPECT_EQ(stamps[i], kLocalStart + static_cast<int64_t>(i) * 66 * kMs + 20 * kMs) << i;
  }
}

TEST(ClockOffset, UnknownCaptureTimeFallsBackToArrival)
{
  ClockOffset clock;
  EXPECT_EQ(clock.toLocal(0, kLocalStart), kLocalStart);
  EXPECT_EQ(clock.toLocal(-5, kLocalStart + 1), kLocalStart + 1);
}

TEST(ClockOffset, RobotClockJumpingForwardKeepsStampsContinuous)
{
  ClockOffset clock;
  clock.toLocal(kRobotStart, kLocalStart + 30 * kMs);
  // The robot's clock steps 1 h ahead between two frames 66 ms apart.
  const int64_t stamp = clock.toLocal(
    kRobotStart + 66 * kMs + 3600 * kSecond, kLocalStart + 96 * kMs);
  EXPECT_EQ(stamp, kLocalStart + 96 * kMs);
}

TEST(ClockOffset, RobotClockJumpingBackRestartsTheHistory)
{
  ClockOffset clock;
  clock.toLocal(kRobotStart, kLocalStart + 30 * kMs);
  // The robot's clock steps 1 h back: without a restart, stamps would be 1 h early.
  const int64_t arrival = kLocalStart + 96 * kMs;
  const int64_t stamp = clock.toLocal(kRobotStart + 66 * kMs - 3600 * kSecond, arrival);
  EXPECT_EQ(stamp, arrival);
}

TEST(ClockOffset, AnOldShortestDelayIsForgottenAfterTheWindow)
{
  ClockOffset clock(/*window_ns=*/ 1 * kSecond);
  clock.toLocal(kRobotStart, kLocalStart + 10 * kMs);  // shortest delay: 10 ms
  // 2 s later every frame takes 50 ms: the stamps follow once 10 ms is out of the window.
  const int64_t capture = kRobotStart + 2 * kSecond;
  const int64_t arrival = kLocalStart + 2 * kSecond + 50 * kMs;
  EXPECT_EQ(clock.toLocal(capture, arrival), arrival);
}
