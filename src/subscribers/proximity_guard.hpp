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

#ifndef PROXIMITY_GUARD_HPP
#define PROXIMITY_GUARD_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace naoqi
{
namespace safety
{

/**
 * @brief A coarse last-resort obstacle stop from Pepper's own laser and sonar.
 *
 * Why it lives in the driver: the laser, the sonar and /cmd_vel all pass
 * through this process, so a check here cannot be orphaned. Nav2's collision
 * monitor runs elsewhere and can be lost -- the Jetson that carries the L2 can
 * drop off, or the network can -- while the wheels still turn. This keeps a
 * coarse obstacle sense as long as the driver itself is alive.
 *
 * Why it is deliberately NOT a copy of Nav2's polygons: these are poor sensors
 * for fine geometry and good ones for "something is close". The laser gives 45
 * sparse beams with two ~27 deg blind wedges, and its message carries the true
 * distance but only a NOMINAL bearing -- the driver folds three boards into one
 * uniform angular grid, so the direction of any one reading is approximate.
 * Asking that data whether a point is inside a hexagon is asking the wrong
 * question of it. Two independent rules also fail independently, where two
 * copies of one polygon share a bug.
 *
 * It only ever SUBTRACTS authority: it clamps a command, never issues one.
 *
 * It fails QUIET, not safe: a stale or absent sensor yields no opinion rather
 * than a stop. That is the opposite of the primary chain, and deliberate --
 * Nav2's monitor plus bumper_guard's required_topics already stop the robot on
 * sensor loss, and a second layer doing the same would double the chance of
 * immobilising the robot for no gain in safety.
 */

enum class Level
{
  kClear = 0,
  kSlow = 1,
  kStop = 2
};

/// Which translation directions are forbidden. Rotation is never gated: the
/// footprint is a ~0.25 m circle, so turning in place sweeps nothing new, and
/// it is how an operator gets the nose off an obstacle.
struct Clamp
{
  bool block_forward = false;
  bool block_reverse = false;
  double scale = 1.0;        // applied to forward speed when not blocked
};

struct Zones
{
  /// Ranges are distances from base_footprint, which is what the laser
  /// publishes (converters/laser.cpp transforms each board before taking the
  /// norm). Defaults sit INSIDE Nav2's 0.40/0.90 m wedges so this never fires
  /// first in normal operation: it is a backstop, not a second opinion.
  double stop_range = 0.35;
  double slow_range = 0.60;
  double slow_scale = 0.3;
  /// Half-angle of the forward arc that matters. Only forward motion is
  /// possible (vx_min 0, vy_max 0), so a return off to the side cannot be
  /// driven into.
  double half_angle = 1.0472;      // 60 deg
  /// Readings needed before believing an obstacle. 1 would make a single
  /// specular glitch stop the robot; the laser has no outlier filter.
  int min_hits = 2;
};

/// Worst level implied by one LaserScan. Ranges must already be distances from
/// base_footprint. Readings outside [range_min, range_max], and the -1.0 that
/// the driver leaves in never-written blind-gap slots, carry no information and
/// are skipped rather than read as clear.
inline Level scanLevel(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Zones & z)
{
  int stop_hits = 0;
  int slow_hits = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = static_cast<double>(ranges[i]);
    if (!std::isfinite(r) || r < range_min || r > range_max) {continue;}
    const double bearing = angle_min + static_cast<double>(i) * angle_increment;
    if (std::fabs(bearing) > z.half_angle) {continue;}
    if (r <= z.stop_range) {
      ++stop_hits;
      ++slow_hits;
    } else if (r <= z.slow_range) {
      ++slow_hits;
    }
  }
  if (stop_hits >= z.min_hits) {return Level::kStop;}
  if (slow_hits >= z.min_hits) {return Level::kSlow;}
  return Level::kClear;
}

/**
 * @brief Level implied by one sonar reading.
 *
 * The sonar publishes in its OWN frame, on the Tibia, which sits forward of
 * base_footprint. An obstacle at range r is therefore further from
 * base_footprint than r, so treating r as a base_footprint distance triggers
 * EARLIER than the truth -- the conservative direction, and it avoids pinning a
 * safety value to a static guess at a chain with three revolute joints.
 *
 * A reading below min_range or above max_range is UNUSABLE, not clear: Pepper's
 * sonar emits values well past its stated maximum (MEASURED 4.77 m against a
 * declared 2.55 m) when it has nothing to report, and its near field is blind.
 * One reading can never reach min_hits on its own, so a sonar stop needs
 * repeated agreement -- see SonarFilter.
 */
inline Level sonarLevel(double range, double min_range, double max_range, const Zones & z)
{
  if (!std::isfinite(range) || range < min_range || range > max_range) {
    return Level::kClear;        // no information; the caller tracks usability
  }
  if (range <= z.stop_range) {return Level::kStop;}
  if (range <= z.slow_range) {return Level::kSlow;}
  return Level::kClear;
}

inline bool sonarUsable(double range, double min_range, double max_range)
{
  return std::isfinite(range) && range >= min_range && range <= max_range;
}

/// Consecutive agreement for the sonar, which has a 30 deg cone and one
/// reading per message: a single echo is not evidence.
class SonarFilter
{
public:
  explicit SonarFilter(int needed)
  : needed_(needed) {}

  void observe(Level l)
  {
    if (l == last_ && l != Level::kClear) {
      ++run_;
    } else {
      run_ = (l == Level::kClear) ? 0 : 1;
    }
    last_ = l;
  }

  Level level() const {return run_ >= needed_ ? last_ : Level::kClear;}

private:
  int needed_;
  int run_ = 0;
  Level last_ = Level::kClear;
};

/// naoqi_bridge_msgs/Bumper::bumper values; BACK exists on Pepper only.
enum BumperId
{
  kRight = 0,
  kLeft = 1,
  kBack = 2,
  kBumperCount = 3
};

/**
 * @brief Latches a bumper contact and blocks only the direction it came from.
 *
 * /bumper is an event topic: naoqi publishes on edges, so gating on the raw
 * state chatters at contact, where a millimetre of rebound reopens the gate.
 * Blocking only one direction matters because the controller runs vx_min 0 and
 * can never reverse, so a contact that zeroes everything leaves Pepper leaning
 * on the obstacle with no command that frees it.
 */
class BumperLatch
{
public:
  explicit BumperLatch(double clear_delay)
  : clear_delay_(clear_delay) {}

  void set(int id, bool pressed, double now_s)
  {
    if (id < 0 || id >= kBumperCount) {return;}
    State & s = st_[id];
    if (pressed) {
      s.pressed = true;
      s.armed = true;
      s.released_at = 0.0;
      s.has_release = false;
    } else if (s.pressed) {
      s.pressed = false;
      s.released_at = now_s;
      s.has_release = true;
    }
  }

  bool latched(int id, double now_s) const
  {
    if (id < 0 || id >= kBumperCount) {return false;}
    const State & s = st_[id];
    if (s.pressed) {return true;}
    if (!s.armed) {return false;}
    if (!s.has_release) {return true;}
    return (now_s - s.released_at) < clear_delay_;
  }

  Clamp clamp(double now_s) const
  {
    Clamp c;
    c.block_forward = latched(kRight, now_s) || latched(kLeft, now_s);
    c.block_reverse = latched(kBack, now_s);
    return c;
  }

  bool any(double now_s) const
  {
    const Clamp c = clamp(now_s);
    return c.block_forward || c.block_reverse;
  }

private:
  struct State
  {
    bool pressed = false;
    bool armed = false;
    bool has_release = false;
    double released_at = 0.0;
  };
  State st_[kBumperCount];
  double clear_delay_;
};

/// Combine a level and a bumper clamp into the clamp actually applied.
inline Clamp combine(Level level, const Clamp & bumper)
{
  Clamp c = bumper;
  if (level == Level::kStop) {
    c.block_forward = true;
  } else if (level == Level::kSlow && !c.block_forward) {
    c.scale = 0.3;
  }
  return c;
}

/// Apply a clamp to a velocity triple. Rotation is untouched by design.
inline void applyClamp(const Clamp & c, float & vx, float & vy, double slow_scale)
{
  if (c.block_forward && vx > 0.f) {vx = 0.f;}
  if (c.block_reverse && vx < 0.f) {vx = 0.f;}
  if (!c.block_forward && c.scale < 1.0 && vx > 0.f) {
    vx = static_cast<float>(vx * slow_scale);
  }
  // vy is zeroed with forward motion: Pepper's base is holonomic hardware even
  // though the controller never commands vy, so a teleop sidestep into a
  // detected obstacle must not slip through.
  if (c.block_forward) {vy = 0.f;}
}

}  // namespace safety
}  // namespace naoqi

#endif
