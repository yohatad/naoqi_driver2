// Safety checks applied to motion commands before they reach ALMotion.

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "subscribers/cmd_vel_watchdog.hpp"
#include "actions/led_validation.hpp"
#include "subscribers/motion_validation.hpp"

using naoqi::subscriber::CmdVelWatchdog;
namespace val = naoqi::subscriber::validation;
using namespace std::chrono_literals;

namespace
{
const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();
}  // namespace

// ---------------------------------------------------------------- watchdog

TEST(CmdVelWatchdog, FiresOnceWhenMovingCommandsStop)
{
  CmdVelWatchdog dog(500ms);
  const auto t0 = CmdVelWatchdog::Clock::time_point{} + 10s;

  dog.onCommand(/*is_zero=*/ false, t0);
  EXPECT_FALSE(dog.expired(t0 + 499ms));
  EXPECT_TRUE(dog.expired(t0 + 500ms));
  EXPECT_FALSE(dog.expired(t0 + 2s));  // already stopped, no repeat
}

TEST(CmdVelWatchdog, FreshCommandsKeepItQuiet)
{
  CmdVelWatchdog dog(500ms);
  auto t = CmdVelWatchdog::Clock::time_point{} + 10s;
  dog.onCommand(false, t);
  for (int i = 0; i < 20; ++i) {
    t += 100ms;  // 10 Hz stream
    dog.onCommand(false, t);
    EXPECT_FALSE(dog.expired(t + 50ms));
  }
}

TEST(CmdVelWatchdog, ZeroCommandMeansNothingToStop)
{
  CmdVelWatchdog dog(500ms);
  const auto t0 = CmdVelWatchdog::Clock::time_point{} + 10s;
  dog.onCommand(/*is_zero=*/ true, t0);
  EXPECT_FALSE(dog.expired(t0 + 5s));
}

TEST(CmdVelWatchdog, RearmsAfterMovingAgain)
{
  CmdVelWatchdog dog(500ms);
  auto t = CmdVelWatchdog::Clock::time_point{} + 10s;
  dog.onCommand(false, t);
  EXPECT_TRUE(dog.expired(t + 1s));
  t += 2s;
  dog.onCommand(false, t);
  EXPECT_TRUE(dog.expired(t + 600ms));
}

TEST(CmdVelWatchdog, NonPositiveTimeoutDisablesIt)
{
  CmdVelWatchdog dog(0ms);
  const auto t0 = CmdVelWatchdog::Clock::time_point{} + 10s;
  EXPECT_FALSE(dog.enabled());
  dog.onCommand(false, t0);
  EXPECT_FALSE(dog.expired(t0 + 1h));
}

TEST(CmdVelWatchdog, NeverFiresBeforeAnyCommand)
{
  CmdVelWatchdog dog(500ms);
  EXPECT_FALSE(dog.expired(CmdVelWatchdog::Clock::now()));
}

// ---------------------------------------------------------------- velocity

TEST(MotionValidation, VelocityRejectsNonFinite)
{
  EXPECT_EQ(val::validateVelocity(0.2, 0.0, 0.1), "");
  EXPECT_NE(val::validateVelocity(kNaN, 0.0, 0.0), "");
  EXPECT_NE(val::validateVelocity(0.0, kInf, 0.0), "");
  EXPECT_NE(val::validateVelocity(0.0, 0.0, -kInf), "");
}

// ---------------------------------------------------------------- setAngles

TEST(MotionValidation, JointAnglesAcceptsChainsAndValidSpeeds)
{
  EXPECT_EQ(val::validateJointAngles({"HeadYaw"}, {0.3f}, 0.1f), "");
  EXPECT_EQ(val::validateJointAngles({"Head"}, {0.3f, -0.1f}, 1.0f), "");  // chain name
}

TEST(MotionValidation, JointAnglesRejectsBadInput)
{
  EXPECT_NE(val::validateJointAngles({}, {0.3f}, 0.1f), "");
  EXPECT_NE(val::validateJointAngles({"HeadYaw"}, {}, 0.1f), "");
  EXPECT_NE(val::validateJointAngles({"HeadYaw"}, {kNaN}, 0.1f), "");
  EXPECT_NE(val::validateJointAngles({"HeadYaw"}, {0.3f}, 0.f), "");
  EXPECT_NE(val::validateJointAngles({"HeadYaw"}, {0.3f}, 1.5f), "");
  EXPECT_NE(val::validateJointAngles({"HeadYaw"}, {0.3f}, kNaN), "");
}

// ---------------------------------------------------------------- trajectories

TEST(MotionValidation, TrajectoryAcceptsSupportedShapes)
{
  // One joint, several waypoints.
  EXPECT_EQ(
    val::validateTrajectory(
      {"HeadYaw"}, {0.1f, 0.2f, 0.0f}, {0.5f, 1.0f, 1.5f}, true,
      false), "");
  // One waypoint per joint.
  EXPECT_EQ(
    val::validateTrajectory(
      {"HeadYaw", "HeadPitch"}, {0.1f, 0.2f}, {1.0f, 0.5f}, true,
      false), "");
  // Bezier, two joints with two waypoints each.
  EXPECT_EQ(
    val::validateTrajectory(
      {"HeadYaw", "HeadPitch"}, {0.1f, 0.2f, 0.3f, 0.4f},
      {0.5f, 1.0f, 0.5f, 1.0f}, true, true), "");
  // Relative is fine without Bezier.
  EXPECT_EQ(val::validateTrajectory({"HeadYaw"}, {0.1f}, {1.0f}, false, false), "");
}

TEST(MotionValidation, TrajectoryRejectsRelativeBezier)
{
  // angleInterpolationBezier is always absolute: relative input would be misread.
  EXPECT_NE(
    val::validateTrajectory(
      {"HeadYaw"}, {0.1f}, {1.0f}, /*is_absolute=*/ false,
      /*use_bezier=*/ true), "");
}

TEST(MotionValidation, TrajectoryRejectsMalformedInput)
{
  EXPECT_NE(val::validateTrajectory({}, {0.1f}, {1.0f}, true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f, 0.2f}, {1.0f}, true, false), "");
  EXPECT_NE(
    val::validateTrajectory(
      {"HeadYaw", "HeadPitch"}, {0.1f, 0.2f, 0.3f}, {1.f, 2.f, 3.f},
      true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {kNaN}, {1.0f}, true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f}, {kInf}, true, false), "");
}

TEST(MotionValidation, TrajectoryRejectsNonIncreasingTimes)
{
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f}, {0.0f}, true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f}, {-1.0f}, true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f, 0.2f}, {1.0f, 1.0f}, true, false), "");
  EXPECT_NE(val::validateTrajectory({"HeadYaw"}, {0.1f, 0.2f}, {1.0f, 0.5f}, true, false), "");
  // Times restart per joint, so a second joint's first waypoint may be earlier.
  EXPECT_EQ(
    val::validateTrajectory(
      {"A", "B"}, {0.f, 0.f, 0.f, 0.f}, {1.f, 2.f, 0.5f, 1.f}, true,
      true), "");
}

// ---------------------------------------------------------------- speed caps

TEST(MotionValidation, LimitVelocityKeepsDirectionAndCapsTurnRate)
{
  const auto v = val::limitVelocity(0.3, 0.4, 2.0, 0.25, 0.5);  // |(x, y)| = 0.5
  EXPECT_NEAR(std::hypot(v.x, v.y), 0.25, 1e-9);
  EXPECT_NEAR(v.x / v.y, 0.3 / 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(v.theta, 0.5);
  EXPECT_DOUBLE_EQ(val::limitVelocity(0.0, 0.0, -2.0, 0.25, 0.5).theta, -0.5);
}

TEST(MotionValidation, LimitVelocityLeavesSlowCommandsAndZeroLimitsAlone)
{
  const auto slow = val::limitVelocity(0.1, 0.0, 0.2, 0.25, 0.5);
  EXPECT_DOUBLE_EQ(slow.x, 0.1);
  EXPECT_DOUBLE_EQ(slow.theta, 0.2);
  const auto unlimited = val::limitVelocity(3.0, 0.0, 9.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(unlimited.x, 3.0);
  EXPECT_DOUBLE_EQ(unlimited.theta, 9.0);
}

// ---------------------------------------------------------------- moveto

TEST(MotionValidation, MoveToAcceptsGoalsWithinTheCap)
{
  EXPECT_EQ(val::validateMoveTo(0.5, -0.5, 3.0), "");
  EXPECT_EQ(val::validateMoveTo(100.0, 0.0, 0.0), "");  // 0 disables the cap
}

TEST(MotionValidation, MoveToRejectsFarAndNonFiniteGoals)
{
  EXPECT_NE(val::validateMoveTo(100.0, 0.0, 3.0), "");
  EXPECT_NE(val::validateMoveTo(2.5, 2.5, 3.0), "");  // 3.54 m away
  EXPECT_NE(val::validateMoveTo(kNaN, 0.0, 3.0), "");
  EXPECT_NE(val::validateMoveTo(0.0, kInf, 0.0), "");
}

// ---------------------------------------------------------------- LEDs

TEST(LedValidation, AcceptsWellFormedGoals)
{
  naoqi_bridge_msgs::action::RunLed::Goal goal;
  goal.mode = goal.MODE_RGB_FADE;
  goal.color.r = 0.f;
  goal.color.g = 0.5f;
  goal.color.b = 1.f;
  goal.duration = 2.f;
  EXPECT_EQ(naoqi::action::validateLedGoal(goal), "");
  goal.mode = goal.MODE_OFF;
  EXPECT_EQ(naoqi::action::validateLedGoal(goal), "");
}

TEST(LedValidation, RejectsOutOfRangeValues)
{
  naoqi_bridge_msgs::action::RunLed::Goal goal;
  goal.mode = goal.MODE_FADE_INTENSITY;
  goal.intensity = 1.5f;
  goal.duration = 1.f;
  EXPECT_NE(naoqi::action::validateLedGoal(goal), "");
  goal.intensity = 0.5f;
  goal.duration = -1.f;
  EXPECT_NE(naoqi::action::validateLedGoal(goal), "");
  goal.duration = kNaN;
  EXPECT_NE(naoqi::action::validateLedGoal(goal), "");
  goal.mode = goal.MODE_RGB;
  goal.color.r = 2.f;
  EXPECT_NE(naoqi::action::validateLedGoal(goal), "");
  goal.mode = 9;
  EXPECT_NE(naoqi::action::validateLedGoal(goal), "");
}
