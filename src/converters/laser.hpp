/*
 * Copyright 2015 Aldebaran
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
 *
*/

#ifndef LASER_CONVERTER_HPP
#define LASER_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>
#include <naoqi_driver/ros_helpers.hpp>

/*
* ROS includes
*/
#include <sensor_msgs/msg/laser_scan.hpp>

/*
* STANDARD includes
*/
#include <cstddef>

namespace naoqi
{
namespace converter
{

/** Number of Seg XY values read per scan (3 banks x 15 segments x 2 axes). */
static const size_t kScanValueCount = 90;

/** Right, Front, Left -- the order they appear in laserMemoryKeys. */
static const size_t kBankCount = 3;

/** A laser board frames at 6.66 Hz (~150 ms). One second is a little over six
 *  frame periods: long enough that a converter polling faster than the boards
 *  does not flicker, short enough to catch a stall quickly. */
static const double kFrameStaleSeconds = 1.0;

class LaserConverter : public BaseConverter<LaserConverter>
{

  typedef boost::function<void(sensor_msgs::msg::LaserScan&)> Callback_t;

public:
  LaserConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session );

  void registerCallback( message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

  void reset( );

  void setLaserRanges(const float &range_min, const float &range_max);

private:

  /** Advance-based liveness for one laser board.
   *
   * The boards stop sampling entirely whenever the robot is at rest, and a
   * stalled board keeps returning its last values forever -- a value stream
   * indistinguishable from an empty room. So a scan being *readable* proves
   * nothing; only a frame counter observed to ADVANCE proves the bank is live.
   *
   * Starts UNPROVEN on purpose. A counter that has never been seen to change
   * is absence of evidence, not evidence of life, so a board that was already
   * stalled when the driver started is caught rather than inherited as
   * healthy. Costs one frame period at startup.
   */
  struct BankLiveness
  {
    float  last_count;
    double last_change_s;
    bool   seen;
    bool   proven;

    BankLiveness() : last_count(0.0f), last_change_s(0.0), seen(false), proven(false) {}

    bool update(float count, double now_s)
    {
      if (!seen)
      {
        seen = true;
        last_count = count;
        last_change_s = now_s;
        return false;
      }
      if (count != last_count)
      {
        last_count = count;
        last_change_s = now_s;
        proven = true;
        return true;
      }
      if (!proven)
      {
        return false;
      }
      return (now_s - last_change_s) < kFrameStaleSeconds;
    }
  };

  qi::AnyObject p_memory_;
  float range_min_;
  float range_max_;

  BankLiveness liveness_[kBankCount];
  double last_warn_s_;

  std::map<message_actions::MessageAction, Callback_t> callbacks_;
  sensor_msgs::msg::LaserScan msg_;
}; // class

} //publisher
} // naoqi

#endif
