/*
 * Copyright 2026 Carnegie Mellon University Africa
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
*/

// Coarse laser/sonar/bumper backstop. Time is passed in, so the cases are
// exact rather than slept through.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "subscribers/proximity_guard.hpp"

using naoqi::safety::BumperLatch;
using naoqi::safety::Clamp;
using naoqi::safety::Level;
using naoqi::safety::SonarFilter;
using naoqi::safety::Zones;
using naoqi::safety::applyClamp;
using naoqi::safety::combine;
using naoqi::safety::kBack;
using naoqi::safety::kLeft;
using naoqi::safety::kRight;
using naoqi::safety::scanLevel;
using naoqi::safety::sonarLevel;
using naoqi::safety::sonarUsable;

namespace
{
constexpr double kAngleMin = -2.0944;              // -120 deg, Pepper's laser
constexpr double kInc = (2 * 2.0944) / 61.0;       // 61 slots
constexpr double kRMin = 0.1;
constexpr double kRMax = 3.0;
const float kInf = std::numeric_limits<float>::infinity();
const float kNan = std::numeric_limits<float>::quiet_NaN();

/// A scan of 61 slots, everything clear, then the caller sets specific slots.
std::vector<float> clearScan() {return std::vector<float>(61, kInf);}

/// Slot index whose nominal bearing is nearest to `deg`.
std::size_t slotFor(double deg)
{
  return static_cast<std::size_t>(std::lround((deg * M_PI / 180.0 - kAngleMin) / kInc));
}

Level scan(const std::vector<float> & r, const Zones & z = Zones())
{
  return scanLevel(r, kAngleMin, kInc, kRMin, kRMax, z);
}
}  // namespace

TEST(ScanLevel, EmptyScanIsClear)
{
  EXPECT_EQ(scan(clearScan()), Level::kClear);
}

TEST(ScanLevel, TwoBeamsInsideStopRangeStop)
{
  auto r = clearScan();
  r[slotFor(0)] = 0.30f;
  r[slotFor(4)] = 0.30f;
  EXPECT_EQ(scan(r), Level::kStop);
}

TEST(ScanLevel, OneBeamIsNotEnough)
{
  // The laser has no outlier filter; a single specular glitch must not stop it.
  auto r = clearScan();
  r[slotFor(0)] = 0.30f;
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, TwoBeamsInSlowBandSlow)
{
  auto r = clearScan();
  r[slotFor(0)] = 0.50f;
  r[slotFor(4)] = 0.50f;
  EXPECT_EQ(scan(r), Level::kSlow);
}

TEST(ScanLevel, BeyondSlowRangeIsClear)
{
  auto r = clearScan();
  r[slotFor(0)] = 0.80f;
  r[slotFor(4)] = 0.80f;
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, SideReturnsAreIgnored)
{
  // vx_min 0 and vy_max 0: a return at 90 deg cannot be driven into.
  auto r = clearScan();
  r[slotFor(90)] = 0.20f;
  r[slotFor(95)] = 0.20f;
  r[slotFor(-90)] = 0.20f;
  r[slotFor(-95)] = 0.20f;
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, BlindGapMinusOneIsNotTreatedAsAnObstacle)
{
  // The driver leaves -1.0 in never-written slots; it is below range_min and
  // carries no information, so it must be skipped, not read as 1 m behind.
  auto r = clearScan();
  for (std::size_t i = 15; i <= 22; ++i) {
    r[i] = -1.0f;
  }
  for (std::size_t i = 38; i <= 45; ++i) {
    r[i] = -1.0f;
  }
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, NanAndInfAreSkipped)
{
  auto r = clearScan();
  r[slotFor(0)] = kNan;
  r[slotFor(4)] = kInf;
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, BeyondRangeMaxIsSkipped)
{
  // A saturation sentinel must never read as a near obstacle.
  auto r = clearScan();
  r[slotFor(0)] = 5.40f;
  r[slotFor(4)] = 7.09f;
  EXPECT_EQ(scan(r), Level::kClear);
}

TEST(ScanLevel, StopWinsOverSlow)
{
  auto r = clearScan();
  r[slotFor(0)] = 0.30f;
  r[slotFor(4)] = 0.30f;
  r[slotFor(8)] = 0.50f;
  r[slotFor(12)] = 0.50f;
  EXPECT_EQ(scan(r), Level::kStop);
}

TEST(ScanLevel, ArcHalfAngleIsHonoured)
{
  Zones z;
  z.half_angle = 0.35;            // ~20 deg
  auto r = clearScan();
  r[slotFor(40)] = 0.20f;
  r[slotFor(44)] = 0.20f;
  EXPECT_EQ(scan(r, z), Level::kClear);
}

TEST(SonarLevelTest, UsabilityBands)
{
  Zones z;
  // Below min_range: blind near field, not "clear floor".
  EXPECT_FALSE(sonarUsable(0.10, 0.25, 2.55));
  // The MEASURED 4.77 m against a declared 2.55 m maximum.
  EXPECT_FALSE(sonarUsable(4.77, 0.25, 2.55));
  EXPECT_TRUE(sonarUsable(1.00, 0.25, 2.55));
  EXPECT_EQ(sonarLevel(4.77, 0.25, 2.55, z), Level::kClear);
  EXPECT_EQ(sonarLevel(0.30, 0.25, 2.55, z), Level::kStop);
  EXPECT_EQ(sonarLevel(0.50, 0.25, 2.55, z), Level::kSlow);
  EXPECT_EQ(sonarLevel(1.50, 0.25, 2.55, z), Level::kClear);
}

TEST(SonarFilterTest, NeedsConsecutiveAgreement)
{
  SonarFilter f(2);
  f.observe(Level::kStop);
  EXPECT_EQ(f.level(), Level::kClear);      // one echo is not evidence
  f.observe(Level::kStop);
  EXPECT_EQ(f.level(), Level::kStop);
}

TEST(SonarFilterTest, ClearResetsTheRun)
{
  SonarFilter f(2);
  f.observe(Level::kStop);
  f.observe(Level::kClear);
  f.observe(Level::kStop);
  EXPECT_EQ(f.level(), Level::kClear);
}

TEST(BumperLatchTest, AnyBumperStopsAllMotion)
{
  for (int id : {kRight, kLeft, kBack}) {
    BumperLatch l;
    l.set(id, true);
    EXPECT_TRUE(l.clamp().stop_all) << "bumper " << id;
  }
}

TEST(BumperLatchTest, ReleaseDoesNotLiftTheStop)
{
  BumperLatch l;
  l.set(kRight, true);
  l.set(kRight, false);                     // released, or a rebound
  EXPECT_TRUE(l.clamp().stop_all);          // only a reset lifts it
}

TEST(BumperLatchTest, BounceKeepsTheStop)
{
  BumperLatch l;
  l.set(kBack, true);
  l.set(kBack, false);
  l.set(kBack, true);                       // ~50 ms bounce, seen on the robot
  l.set(kBack, false);
  EXPECT_TRUE(l.clamp().stop_all);
}

TEST(BumperLatchTest, ClearRefusedWhilePressed)
{
  BumperLatch l;
  l.set(kLeft, true);
  EXPECT_FALSE(l.clear());
  EXPECT_TRUE(l.clamp().stop_all);
}

TEST(BumperLatchTest, ClearAfterReleaseLiftsTheStop)
{
  BumperLatch l;
  l.set(kLeft, true);
  l.set(kLeft, false);
  EXPECT_TRUE(l.clear());
  EXPECT_FALSE(l.clamp().stop_all);
  EXPECT_FALSE(l.any());
}

TEST(BumperLatchTest, ClearRefusedWhileAnotherBumperIsPressed)
{
  BumperLatch l;
  l.set(kRight, true);
  l.set(kRight, false);
  l.set(kBack, true);
  EXPECT_FALSE(l.clear());
  EXPECT_TRUE(l.clamp().stop_all);
}

TEST(BumperLatchTest, OutOfRangeIdIgnored)
{
  BumperLatch l;
  l.set(7, true);
  l.set(-1, true);
  EXPECT_FALSE(l.any());
}

TEST(Combine, StopBlocksForwardAndSlowScales)
{
  Clamp none;
  EXPECT_TRUE(combine(Level::kStop, none).block_forward);
  EXPECT_LT(combine(Level::kSlow, none).scale, 1.0);
  EXPECT_FALSE(combine(Level::kSlow, none).block_forward);
  EXPECT_DOUBLE_EQ(combine(Level::kClear, none).scale, 1.0);
}

TEST(ApplyClamp, BlocksForwardKeepsReverseAndRotation)
{
  Clamp c;
  c.block_forward = true;
  float vx = 0.35f;
  float vy = 0.10f;
  float vth = 0.4f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_FLOAT_EQ(vx, 0.0f);
  EXPECT_FLOAT_EQ(vy, 0.0f);               // holonomic hardware; no sidestep in
  vx = -0.15f;
  vy = 0.0f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_FLOAT_EQ(vx, -0.15f);             // the escape survives
  EXPECT_FLOAT_EQ(vth, 0.4f);              // the laser never gates rotation
}

TEST(ApplyClamp, SlowScalesForwardOnly)
{
  Clamp c = combine(Level::kSlow, Clamp());
  float vx = 0.30f;
  float vy = 0.0f;
  float vth = 0.0f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_NEAR(vx, 0.09f, 1e-6);
  vx = -0.20f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_FLOAT_EQ(vx, -0.20f);             // reverse is not scaled
}

TEST(ApplyClamp, BackBumperBlocksReverseOnly)
{
  Clamp c;
  c.block_reverse = true;
  float vx = -0.20f;
  float vy = 0.0f;
  float vth = 0.0f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_FLOAT_EQ(vx, 0.0f);
  vx = 0.20f;
  applyClamp(c, vx, vy, vth, 0.3);
  EXPECT_FLOAT_EQ(vx, 0.20f);
}

TEST(ApplyClamp, BumperStopZeroesEveryAxis)
{
  BumperLatch l;
  l.set(kRight, true);
  const Clamp c = combine(Level::kClear, l.clamp());
  for (float sign : {1.f, -1.f}) {
    float vx = 0.3f * sign;
    float vy = 0.1f * sign;
    float vth = 0.5f * sign;
    applyClamp(c, vx, vy, vth, 0.3);
    EXPECT_FLOAT_EQ(vx, 0.0f);
    EXPECT_FLOAT_EQ(vy, 0.0f);
    EXPECT_FLOAT_EQ(vth, 0.0f);             // no turning either
  }
}

TEST(Defaults, BackstopSitsInsideNav2Zones)
{
  // It must not fire before Nav2's 0.40/0.90 m wedges in normal operation.
  Zones z;
  EXPECT_LT(z.stop_range, 0.40);
  EXPECT_LT(z.slow_range, 0.90);
  EXPECT_GE(z.min_hits, 2);
}
