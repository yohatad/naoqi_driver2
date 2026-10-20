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

/*
 * LOCAL includes
 */
#include "teleop.hpp"

#include <naoqi_driver/ros_helpers.hpp>
#include "motion_validation.hpp"
#include "helpers/async_helpers.hpp"
#include "helpers/parameter_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber(
  const std::string & name,
  const std::string & cmd_vel_topic,
  const std::string & joint_angles_topic,
  const std::string & joint_angle_traj_topic,
  const qi::SessionPtr & session)
: cmd_vel_topic_(cmd_vel_topic)
  , joint_angles_topic_(joint_angles_topic)
  , joint_angle_traj_topic_(joint_angle_traj_topic)
  , BaseSubscriber(name, cmd_vel_topic, session)
  , p_motion_(session->service("ALMotion").value())
{}

void TeleopSubscriber::reset(rclcpp::Node * node)
{
  logger_ = node->get_logger();
  // Motion commands and the watchdog get their own callback group, so a slow service
  // or action callback elsewhere in the driver never delays them.
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  const double timeout_s = helpers::declareNumberParameter(
    node, "cmd_vel_timeout", 0.5,
    "Seconds without /cmd_vel before a moving base is stopped; 0 disables");
  max_linear_speed_ = helpers::declareNumberParameter(
    node, "max_linear_speed", 0.0, "Cap on the /cmd_vel linear speed in m/s; 0 means no cap");
  max_angular_speed_ = helpers::declareNumberParameter(
    node, "max_angular_speed", 0.0, "Cap on the /cmd_vel angular speed in rad/s; 0 means no cap");
  cmd_vel_watchdog_ = std::make_shared<CmdVelWatchdog>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timeout_s)));
  cmd_vel_watchdog_timer_.reset();
  if (cmd_vel_watchdog_->enabled()) {
    cmd_vel_watchdog_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100), [this]()
      {
        if (cmd_vel_watchdog_->expired(CmdVelWatchdog::Clock::now())) {
          RCLCPP_WARN(logger_, "No /cmd_vel for the configured timeout, stopping the base");
          helpers::warnOnAsyncError(
            p_motion_.async<void>("move", 0.0f, 0.0f, 0.0f), logger_, "ALMotion.move (stop)");
        }
      }, callback_group_);
  }
  // Coarse laser/sonar/bumper backstop. ON by default: a safety layer that has
  // to be asked for is one that is missing when it matters, and opting in was
  // measured to fail exactly that way -- launched without a params file, the
  // guard never started, so /bumper had no subscriber at all and pressing a
  // bumper on a robot driving forward did nothing. It fails quiet when a source
  // is stale, so enabling it with no laser or sonar costs nothing.
  // reset() can run again on the same node; declaring twice throws.
  if (!node->has_parameter("proximity_guard")) {
    node->declare_parameter<bool>("proximity_guard", true);
  }
  proximity_enabled_ = node->get_parameter("proximity_guard").as_bool();
  zones_.stop_range = helpers::declareNumberParameter(
    node, "proximity_stop_range", zones_.stop_range,
    "Metres from base_footprint; a forward obstacle inside this blocks forward motion");
  zones_.slow_range = helpers::declareNumberParameter(
    node, "proximity_slow_range", zones_.slow_range,
    "Metres from base_footprint; a forward obstacle inside this scales forward motion");
  zones_.half_angle = helpers::declareNumberParameter(
    node, "proximity_half_angle", zones_.half_angle,
    "Half-angle in radians of the forward arc that is checked");
  zones_.min_hits = static_cast<int>(helpers::declareNumberParameter(
      node, "proximity_min_hits", zones_.min_hits,
      "Readings needed before believing an obstacle; 1 lets one glitch stop the robot"));
  source_timeout_ = helpers::declareNumberParameter(
    node, "proximity_source_timeout", source_timeout_,
    "Seconds before a laser or sonar reading stops counting");
  const double clear_delay = helpers::declareNumberParameter(
    node, "proximity_bumper_clear_delay", 1.0,
    "Seconds a bumper latch outlives the switch release");
  bumper_latch_ = std::make_shared<safety::BumperLatch>(clear_delay);
  sonar_filter_ = safety::SonarFilter(std::max(2, zones_.min_hits));

  sub_laser_.reset();
  sub_sonar_.reset();
  sub_bumper_.reset();
  proximity_timer_.reset();
  if (proximity_enabled_) {
    diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    // SensorDataQoS is BEST_EFFORT, which matches both reliable and
    // best-effort publishers; a reliable subscription here would silently
    // receive nothing from a best-effort sensor.
    sub_laser_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {laserCallback(m);}, options);
    sub_sonar_ = node->create_subscription<sensor_msgs::msg::Range>(
      "/sonar/front", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Range::SharedPtr m) {sonarCallback(m);}, options);
    // Events are rare but must not be dropped, so this queue is deep.
    sub_bumper_ = node->create_subscription<naoqi_bridge_msgs::msg::Bumper>(
      "/bumper", 50,
      [this](const naoqi_bridge_msgs::msg::Bumper::SharedPtr m) {bumperCallback(m);}, options);
    proximity_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {proximityTick();}, callback_group_);
    RCLCPP_INFO(
      logger_,
      "proximity backstop ON: stop %.2f m, slow %.2f m, arc +-%.0f deg, min_hits %d",
      zones_.stop_range, zones_.slow_range, zones_.half_angle * 180.0 / M_PI,
      zones_.min_hits);
  }

  RCLCPP_INFO(
    logger_, "cmd_vel_timeout: %.2f s%s", timeout_s,
    cmd_vel_watchdog_->enabled() ? "" : " (disabled)");

  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10,
    std::bind(&TeleopSubscriber::cmdVelCallback, this, std::placeholders::_1), options);

  sub_joint_angles_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
    joint_angles_topic_, 10,
    std::bind(&TeleopSubscriber::jointAnglesCallback, this, std::placeholders::_1), options);

  sub_joint_angle_traj_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesTrajectory>(
    joint_angle_traj_topic_, 10,
    std::bind(&TeleopSubscriber::jointAngleTrajCallback, this, std::placeholders::_1), options);

  is_initialized_ = true;
}

void TeleopSubscriber::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  const std::string error = validation::validateVelocity(
    twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring /cmd_vel: %s", error.c_str());
    return;
  }

  const auto limited = validation::limitVelocity(
    twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z,
    max_linear_speed_, max_angular_speed_);
  float vel_x = limited.x;
  float vel_y = limited.y;
  const float vel_th = limited.theta;

  if (proximity_enabled_) {
    const safety::Clamp clamp = currentClamp();
    const float was_x = vel_x;
    safety::applyClamp(clamp, vel_x, vel_y, zones_.slow_scale);
    if (vel_x != was_x) {
      // Not a 'return': ALMotion.move() latches its last velocity, so dropping
      // the command would leave the robot driving on the previous one. The
      // clamped value has to be sent.
      RCLCPP_WARN_THROTTLE(
        logger_, *rclcpp::Clock::make_shared(), 2000,
        "proximity backstop: vx %.2f -> %.2f", was_x, vel_x);
    }
  }

  RCLCPP_DEBUG(logger_, "going to move x: %f y: %f th: %f", vel_x, vel_y, vel_th);
  if (cmd_vel_watchdog_) {
    cmd_vel_watchdog_->onCommand(
      vel_x == 0.f && vel_y == 0.f && vel_th == 0.f,
      CmdVelWatchdog::Clock::now());
  }
  {
    std::lock_guard<std::mutex> lock(proximity_mutex_);
    moving_ = !(vel_x == 0.f && vel_y == 0.f && vel_th == 0.f);
  }
  helpers::warnOnAsyncError(
    p_motion_.async<void>("move", vel_x, vel_y, vel_th), logger_, "ALMotion.move");
}

// ---------------------------------------------------------------------------
// Coarse laser/sonar/bumper backstop. See proximity_guard.hpp for the rationale.
// ---------------------------------------------------------------------------

void TeleopSubscriber::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // The driver already transformed every board into base_footprint before
  // taking the norm, so these ranges are distances from the robot origin.
  const safety::Level l = safety::scanLevel(
    msg->ranges, msg->angle_min, msg->angle_increment,
    msg->range_min, msg->range_max, zones_);
  std::lock_guard<std::mutex> lock(proximity_mutex_);
  laser_level_ = l;
  laser_stamp_ = helpers::Time::now().seconds();
}

void TeleopSubscriber::sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(proximity_mutex_);
  // An unusable reading is not evidence of clear floor: Pepper's sonar emits
  // values past its declared maximum when it has nothing to report, and its
  // near field is blind. Leave the level alone and let it age out instead.
  if (!safety::sonarUsable(msg->range, msg->min_range, msg->max_range)) {
    return;
  }
  sonar_filter_.observe(safety::sonarLevel(msg->range, msg->min_range, msg->max_range, zones_));
  sonar_level_ = sonar_filter_.level();
  sonar_stamp_ = helpers::Time::now().seconds();
}

void TeleopSubscriber::bumperCallback(const naoqi_bridge_msgs::msg::Bumper::SharedPtr msg)
{
  const bool pressed = msg->state == naoqi_bridge_msgs::msg::Bumper::STATE_PRESSED;
  std::lock_guard<std::mutex> lock(proximity_mutex_);
  bumper_latch_->set(msg->bumper, pressed, helpers::Time::now().seconds());
  if (pressed) {
    RCLCPP_WARN(logger_, "bumper %u pressed; blocking that direction", msg->bumper);
  }
}

safety::Clamp TeleopSubscriber::currentClamp()
{
  const double now_s = helpers::Time::now().seconds();
  std::lock_guard<std::mutex> lock(proximity_mutex_);
  // A stale source yields no opinion rather than a stop: this is a backstop,
  // and the primary chain already fails safe on sensor loss.
  safety::Level worst = safety::Level::kClear;
  if (laser_stamp_ > 0.0 && now_s - laser_stamp_ <= source_timeout_) {
    worst = std::max(worst, laser_level_);
  }
  if (sonar_stamp_ > 0.0 && now_s - sonar_stamp_ <= source_timeout_) {
    worst = std::max(worst, sonar_level_);
  }
  return safety::combine(worst, bumper_latch_->clamp(now_s));
}

void TeleopSubscriber::proximityTick()
{
  const safety::Clamp clamp = currentClamp();
  bool should_stop;
  bool moving;
  {
    std::lock_guard<std::mutex> lock(proximity_mutex_);
    moving = moving_;
    should_stop = clamp.block_forward;
    if (!should_stop) {
      stop_sent_ = false;
    }
  }
  // ALMotion.move() holds its last velocity, so an obstacle appearing after
  // the last command -- or while /cmd_vel has gone quiet -- needs a zero
  // pushed. Once per detection, like the cmd_vel watchdog.
  if (should_stop && moving) {
    bool send = false;
    {
      std::lock_guard<std::mutex> lock(proximity_mutex_);
      send = !stop_sent_;
      stop_sent_ = true;
      if (send) {
        moving_ = false;
      }
    }
    if (send) {
      RCLCPP_WARN(logger_, "proximity backstop: obstacle ahead, stopping the base");
      helpers::warnOnAsyncError(
        p_motion_.async<void>("move", 0.0f, 0.0f, 0.0f), logger_,
        "ALMotion.move (proximity stop)");
    }
  }

  safety::Level level = safety::Level::kClear;
  if (clamp.block_forward) {
    level = safety::Level::kStop;
  } else if (clamp.scale < 1.0) {
    level = safety::Level::kSlow;
  }
  publishProximityDiagnostics(level, clamp);
}

void TeleopSubscriber::publishProximityDiagnostics(
  safety::Level level, const safety::Clamp & clamp)
{
  if (!diag_pub_) {
    return;
  }
  const double now_s = helpers::Time::now().seconds();
  bool laser_fresh;
  bool sonar_fresh;
  {
    std::lock_guard<std::mutex> lock(proximity_mutex_);
    laser_fresh = laser_stamp_ > 0.0 && now_s - laser_stamp_ <= source_timeout_;
    sonar_fresh = sonar_stamp_ > 0.0 && now_s - sonar_stamp_ <= source_timeout_;
  }
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "naoqi_driver: proximity backstop";
  st.hardware_id = "pepper_laser_sonar_bumper";
  if (level == safety::Level::kStop) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "obstacle ahead; forward motion blocked";
  } else if (level == safety::Level::kSlow) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "obstacle in the slow band; forward motion scaled";
  } else if (!laser_fresh && !sonar_fresh) {
    // Reported, not acted on: a backstop that immobilises the robot when its
    // own sensors are quiet would be worse than no backstop.
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "no fresh laser or sonar; backstop has no opinion";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "clear";
  }
  const auto kv = [&st](const char * k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    };
  kv("laser_fresh", laser_fresh ? "true" : "false");
  kv("sonar_fresh", sonar_fresh ? "true" : "false");
  kv("block_forward", clamp.block_forward ? "true" : "false");
  kv("block_reverse", clamp.block_reverse ? "true" : "false");
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = helpers::Time::now();
  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

void TeleopSubscriber::jointAnglesCallback(
  const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg)
{
  const std::string error = validation::validateJointAngles(
    js_msg->joint_names, js_msg->joint_angles, js_msg->speed);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring JointAnglesWithSpeed: %s", error.c_str());
    return;
  }

  if (js_msg->relative == 0) {
    helpers::warnOnAsyncError(
      p_motion_.async<void>("setAngles", js_msg->joint_names, js_msg->joint_angles, js_msg->speed),
      logger_, "ALMotion.setAngles");
  } else {
    helpers::warnOnAsyncError(
      p_motion_.async<void>(
        "changeAngles", js_msg->joint_names, js_msg->joint_angles,
        js_msg->speed),
      logger_, "ALMotion.changeAngles");
  }
}

void TeleopSubscriber::jointAngleTrajCallback(
  const naoqi_bridge_msgs::msg::JointAnglesTrajectory::SharedPtr traj_msg)
{
  // Basic validation
  const auto & names = traj_msg->joint_names;
  const auto & angles = traj_msg->joint_angles;
  const auto & times = traj_msg->times;

  if (names.empty() || angles.empty() || times.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: empty fields (names:%zu angles:%zu times:%zu)",
      names.size(), angles.size(), times.size());
    return;
  }

  if (angles.size() != times.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: angles.size() (%zu) != times.size() (%zu)",
      angles.size(), times.size());
    return;
  }

  bool is_absolute = (traj_msg->relative == 0);
  bool use_bezier = traj_msg->use_bezier;  // NEW: Check for Bezier flag

  const std::string error = validation::validateTrajectory(
    names, angles, times, is_absolute,
    use_bezier);
  if (!error.empty()) {
    RCLCPP_WARN(logger_, "Ignoring JointAnglesTrajectory: %s", error.c_str());
    return;
  }

  try {
    if (use_bezier) {
      // Use Bezier interpolation for smooth biological motion
      executeBezierTrajectory(names, angles, times);
    } else {
      // Original angleInterpolation logic
      executeAngleInterpolation(names, angles, times, is_absolute);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("TeleopSubscriber"),
      "ALMotion trajectory error: %s", e.what());
  }
}

void TeleopSubscriber::executeAngleInterpolation(
  const std::vector<std::string> & names,
  const std::vector<float> & angles,
  const std::vector<float> & times,
  bool is_absolute)
{
  if (names.size() == 1) {
    // Single joint/chain with multiple waypoints
    helpers::warnOnAsyncError(
      p_motion_.async<void>("angleInterpolation", names[0], angles, times, is_absolute), logger_,
      "ALMotion.angleInterpolation");
  } else if (names.size() == angles.size()) {
    // One waypoint per joint -> all reach their target at the same per-joint time
    std::vector<std::vector<float>> angleLists;
    std::vector<std::vector<float>> timeLists;
    angleLists.reserve(names.size());
    timeLists.reserve(names.size());

    for (size_t i = 0; i < names.size(); ++i) {
      angleLists.push_back(std::vector<float>{angles[i]});
      timeLists.push_back(std::vector<float>{times[i]});
    }
    helpers::warnOnAsyncError(
      p_motion_.async<void>(
        "angleInterpolation", names, angleLists, timeLists,
        is_absolute), logger_,
      "ALMotion.angleInterpolation");
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "JointAnglesTrajectory: unsupported combination: names:%zu angles:%zu times:%zu",
      names.size(), angles.size(), times.size());
  }
}

void TeleopSubscriber::executeBezierTrajectory(
  const std::vector<std::string> & names,
  const std::vector<float> & angles,
  const std::vector<float> & times)
{
  // angleInterpolationBezier expects:
  // - jointNames: vector<string>
  // - timeLists: list of lists (ragged array)
  // - angleLists: list of lists (ragged array)

  // Determine waypoints per joint
  size_t waypoints_per_joint = angles.size() / names.size();

  if (angles.size() % names.size() != 0 || times.size() != angles.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("TeleopSubscriber"),
      "Bezier: invalid dimensions - names:%zu angles:%zu times:%zu",
      names.size(), angles.size(), times.size());
    return;
  }

  // Reshape flat arrays into ragged arrays
  std::vector<std::vector<float>> timeLists;
  std::vector<std::vector<float>> angleLists;
  timeLists.reserve(names.size());
  angleLists.reserve(names.size());

  for (size_t i = 0; i < names.size(); ++i) {
    std::vector<float> joint_angles;
    std::vector<float> joint_times;
    joint_angles.reserve(waypoints_per_joint);
    joint_times.reserve(waypoints_per_joint);

    for (size_t wp = 0; wp < waypoints_per_joint; ++wp) {
      joint_angles.push_back(angles[i * waypoints_per_joint + wp]);
      joint_times.push_back(times[i * waypoints_per_joint + wp]);
    }

    angleLists.push_back(joint_angles);
    timeLists.push_back(joint_times);
  }

  helpers::warnOnAsyncError(
    p_motion_.async<void>("angleInterpolationBezier", names, timeLists, angleLists), logger_,
    "ALMotion.angleInterpolationBezier");

  RCLCPP_INFO(
    rclcpp::get_logger("TeleopSubscriber"),
    "Bezier trajectory: %zu joint%s with %zu waypoint%s each",
    names.size(), names.size() == 1 ? "" : "s",
    waypoints_per_joint, waypoints_per_joint == 1 ? "" : "s");
}

}  // namespace subscriber
}  // namespace naoqi
