/*
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

#ifndef CLOCK_OFFSET_HPP
#define CLOCK_OFFSET_HPP

#include <cstdint>
#include <deque>

namespace naoqi
{
namespace helpers
{

/**
 * Maps capture times on the robot's clock to the local clock.
 *
 * arrival - capture is the offset between the two clocks plus the delay of that
 * sample (waiting in NAOqi, transfer). The smallest value seen recently is the
 * offset plus the shortest delay, so capture + that minimum keeps the spacing
 * between captures exact, is late by the shortest delay at most, and is never
 * later than the arrival. A difference more than max_delay above the minimum is
 * taken as a jump of one of the clocks, and the history restarts.
 */
class ClockOffset
{
public:
  explicit ClockOffset(
    int64_t window_ns = 10'000'000'000, int64_t max_delay_ns = 1'000'000'000)
  : window_ns_(window_ns), max_delay_ns_(max_delay_ns) {}

  /** Local time of a capture at robot_ns (0: unknown) that arrived at arrival_ns. */
  int64_t toLocal(int64_t robot_ns, int64_t arrival_ns)
  {
    if (robot_ns <= 0) {
      return arrival_ns;
    }
    const int64_t offset = arrival_ns - robot_ns;
    if (!samples_.empty() && offset - samples_.front().offset > max_delay_ns_) {
      samples_.clear();
    }
    // Sliding-window minimum: offsets increase from front to back.
    while (!samples_.empty() && samples_.back().offset >= offset) {
      samples_.pop_back();
    }
    samples_.push_back({arrival_ns, offset});
    while (samples_.front().arrival_ns < arrival_ns - window_ns_) {
      samples_.pop_front();
    }
    return robot_ns + samples_.front().offset;
  }

private:
  struct Sample
  {
    int64_t arrival_ns;
    int64_t offset;
  };

  int64_t window_ns_;
  int64_t max_delay_ns_;
  std::deque<Sample> samples_;
};

}  // namespace helpers
}  // namespace naoqi

#endif
