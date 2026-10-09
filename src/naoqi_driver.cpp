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
 * BOOST
 */
#include <memory>
#include <functional>
#include <boost/property_tree/json_parser.hpp>

/*
 * ROS
 */
#include <tf2_ros/buffer.h>

/*
 * PUBLIC INTERFACE
 */
#include <naoqi_driver/naoqi_driver.hpp>
#include <naoqi_driver/message_actions.h>
#include <naoqi_driver/tools.hpp>

/*
 * CONVERTERS
 */
#include "converters/audio.hpp"
#include "converters/touch.hpp"
#include "converters/camera.hpp"
#include "converters/diagnostics.hpp"
#include "converters/imu.hpp"
#include "converters/info.hpp"
#include "converters/joint_state.hpp"
#include "converters/laser.hpp"
#include "converters/memory_list.hpp"
#include "converters/sonar.hpp"
#include "converters/memory/bool.hpp"
#include "converters/memory/float.hpp"
#include "converters/memory/int.hpp"
#include "converters/memory/string.hpp"
#include "converters/log.hpp"
#include "converters/odom.hpp"

/*
 * PUBLISHERS
 */
#include "publishers/basic.hpp"
#include "publishers/camera.hpp"
#include "publishers/info.hpp"
#include "publishers/joint_state.hpp"
#include "publishers/log.hpp"
#include "publishers/sonar.hpp"

/*
 * TOOLS
 */
#include "tools/robot_description.hpp"
#include "tools/alvisiondefinitions.h" // for kTop...

/*
 * SUBSCRIBERS
 */
#include "subscribers/teleop.hpp"
#include "subscribers/moveto.hpp"
#include "subscribers/speech.hpp"

/*
 * SERVICES
 */
#include "services/robot_config.hpp"
#include "services/set_language.hpp"
#include "services/get_language.hpp"
#include "services/odom_reset.hpp"
#include "services/trigger.hpp"

/*
 * RECORDERS
 */
#include "recorder/basic.hpp"
#include "recorder/basic_event.hpp"
#include "recorder/camera.hpp"
#include "recorder/diagnostics.hpp"
#include "recorder/joint_state.hpp"
#include "recorder/sonar.hpp"

/*
 * EVENTS
 */
#include "event/basic.hpp"
#include "event/audio.hpp"
#include "event/touch.hpp"

/*
 * ACTIONS
 */
#include "actions/listen.hpp"
#include "actions/speech.hpp"
#include "actions/led.hpp"
#include "actions/audio_player.hpp"

/*
 * STATIC FUNCTIONS INCLUDE
 */
#include "heartbeat.hpp"
#include "helpers/filesystem_helpers.hpp"
#include "helpers/recorder_helpers.hpp"
#include "helpers/naoqi_helpers.hpp"
#include "helpers/driver_helpers.hpp"
#include "helpers/parameter_helpers.hpp"


namespace naoqi
{

namespace
{

/**
 * Spins the node's callbacks on two threads until destroyed. With callback groups,
 * a slow service call cannot hold up /cmd_vel and its watchdog.
 */
class CallbackThread
{
public:
  explicit CallbackThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  : node_(std::move(node)), executor_(rclcpp::ExecutorOptions(), 2)
  {
    executor_.add_node(node_);
    thread_ = std::thread(
      [this] {
        executor_.spin();
        done_ = true;
      });
  }

  ~CallbackThread()
  {
    // cancel() can race with spin() starting, so repeat it until spin() returned.
    while (!done_) {
      executor_.cancel();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    thread_.join();
    executor_.remove_node(node_);
  }

  CallbackThread(const CallbackThread &) = delete;
  CallbackThread & operator=(const CallbackThread &) = delete;

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread thread_;
  std::atomic<bool> done_{false};
};

}  // namespace

namespace
{
using ImuPublisher = publisher::BasicPublisher<sensor_msgs::msg::Imu>;
using ImuRecorder = recorder::BasicRecorder<sensor_msgs::msg::Imu>;
using LaserPublisher = publisher::BasicPublisher<sensor_msgs::msg::LaserScan>;
using LaserRecorder = recorder::BasicRecorder<sensor_msgs::msg::LaserScan>;
using OdomPublisher = publisher::BasicPublisher<nav_msgs::msg::Odometry>;
using OdomRecorder = recorder::BasicRecorder<nav_msgs::msg::Odometry>;
using DiagnosticsPublisher = publisher::BasicPublisher<diagnostic_msgs::msg::DiagnosticArray>;
using InfoRecorder = recorder::BasicRecorder<naoqi_bridge_msgs::msg::StringStamped>;
using MemoryListPublisher = publisher::BasicPublisher<naoqi_bridge_msgs::msg::MemoryList>;
using MemoryListRecorder = recorder::BasicRecorder<naoqi_bridge_msgs::msg::MemoryList>;
}  // namespace

Driver::Driver()
: rclcpp::Node("naoqi_driver"),
  freq_(15),
  publish_enabled_(false),
  record_enabled_(false),
  log_enabled_(false),
  keep_looping_(true),
  recorder_(std::make_shared<recorder::GlobalRecorder>(get_namespace())),
  buffer_duration_(helpers::recorder::bufferDefaultDuration) {}

Driver::~Driver() = default;

void Driver::run()
{
  loadBootConfig();
  auto robot_desc_pub = tools::publishRobotDescription(this, robot_);
  registerDefaultConverter();
  registerDefaultSubscriber();
  registerDefaultServices();

  // Keeps the last minidump_duration seconds of sensor data for /naoqi_driver/minidump.
  const double minidump_duration = helpers::declareNumberParameter(
    this, "minidump_duration", 0.0,
    "Seconds of recent sensor data kept in memory for /naoqi_driver/minidump; 0 turns it off");
  if (minidump_duration > 0.0) {
    setBufferDuration(static_cast<float>(minidump_duration));
    startLogging();
  }

  // Action servers. They outlive the callback thread below; destroying them stops
  // their running goals in NAOqi and joins the goal threads.
  std::vector<std::unique_ptr<action::ActionHandle>> actions;
  actions.push_back(action::createListenServer(this, session_));
  actions.push_back(action::createSpeechWithFeedbackServer(this, session_));
  actions.push_back(action::createAudioPlayerServers(this, session_));
  actions.push_back(action::createRunLedServer(this, session_));

  // A single iteration will propagate registrations, etc...
  rosIteration();

  {
    std::lock_guard<std::mutex> lock(mutex_conv_queue_);

    if (converters_.empty()) {
      // If there is no converters, create them
      // (converters only depends on Naoqi, resetting the
      // Ros node has no impact on them)
      RCLCPP_INFO_STREAM(get_logger(), "going to register converters");
      registerDefaultConverter();
      registerDefaultSubscriber();
    } else {
      RCLCPP_INFO_STREAM(get_logger(), "NOT going to re-register the converters");
      // If some converters are already there, then
      // we just need to reset the registered publisher
      // using the ROS node
      for (auto & [name, pub] : pub_map_) {
        pub->reset(this);
      }

      for (auto & sub : subscribers_) {
        sub->reset(this);
      }

      for (auto & srv : services_) {
        srv->reset(this);
      }
    }

    if (!event_map_.empty()) {
      for (auto & [name, event] : event_map_) {
        event->resetPublisher(this);
      }
    }
    // Start publishing again
    startPublishing();
  }

  RCLCPP_INFO_STREAM(get_logger(), "naoqi_driver initialized");
  RCLCPP_INFO_STREAM(get_logger(), "Starting ROS loop");

  // Lets scripts/robot_watchdog.py, on the robot, stop the base if this process dies,
  // hangs or loses the network. It beats from the callback threads below.
  Heartbeat heartbeat(this, session_, std::chrono::milliseconds(200));

  // ROS callbacks (commands, services, actions, timers) run on their own threads,
  // so they never wait for the converter schedule of this loop.
  CallbackThread callbacks(get_node_base_interface());
  while (keep_looping_) {
    this->rosIteration();
  }
}

/**
 * Sets the Qi Session to use,
 * sets robot and has_stereo_ objects.
 * Must be called before the run function.
 *
 * @param session
 */
void Driver::setQiSession(const qi::SessionPtr & session)
{
  this->session_ = session;
  robot_ = helpers::driver::getRobot(session);
  has_stereo_ = helpers::driver::isDepthStereo(session);
}

void Driver::loadBootConfig()
{
  const std::string & file_path = helpers::filesystem::getBootConfigFile();
  RCLCPP_INFO_STREAM(get_logger(), "load boot config from " << file_path);
  boost::property_tree::ptree tree;
  if (!file_path.empty()) {
    boost::property_tree::read_json(file_path, tree);
  }
  // Parameters given to the node override the file.
  boot_config_ = declareBootConfigParameters(*this, parseBootConfig(tree));
}

void Driver::rosIteration()
{
  std::vector<message_actions::MessageAction> actions;

  {
    std::lock_guard<std::mutex> lock(mutex_conv_queue_);
    if (!conv_queue_.empty()) {
      // Wait for the next Publisher to be ready
      size_t conv_index = conv_queue_.top().conv_index_;
      const auto & conv = converters_[conv_index];
      rclcpp::Time schedule = conv_queue_.top().schedule_;

      // check the publishing condition
      // 1. publishing enabled
      // 2. has to be registered
      // 3. has to be subscribed
      auto pub_it = pub_map_.find(conv->name() );
      if (publish_enabled_ && pub_it != pub_map_.end() && pub_it->second->isSubscribed() ) {
        actions.push_back(message_actions::PUBLISH);
      }

      // check the recording condition
      // 1. recording enabled
      // 2. has to be registered
      // 3. has to be subscribed (configured to be recorded)
      auto rec_it = rec_map_.find(conv->name() );
      {
        std::unique_lock<std::mutex> lock_record(mutex_record_, std::try_to_lock);
        if (lock_record && record_enabled_ && rec_it != rec_map_.end() &&
          rec_it->second->isSubscribed() )
        {
          actions.push_back(message_actions::RECORD);
        }
      }

      // bufferize data in recorder
      if (log_enabled_ && rec_it != rec_map_.end() && conv->frequency() != 0) {
        actions.push_back(message_actions::LOG);
      }

      // only call when we have at least one action to perform
      if (!actions.empty()) {
        // A NAOqi error in one converter (camera busy, network hiccup) must not
        // take the whole driver down; it is retried on its next schedule.
        try {
          conv->callAll(actions);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Converter %s failed: %s", conv->name().c_str(), e.what());
        }
      }

      rclcpp::Duration d(schedule - this->now());
      if (d > rclcpp::Duration(0, 0)) {
        rclcpp::sleep_for(d.to_chrono<std::chrono::nanoseconds>());
      }

      // Schedule for a future time or not
      conv_queue_.pop();
      if (conv->frequency() != 0) {
        conv_queue_.push(
          ScheduledConverter(
            schedule +
            rclcpp::Duration(0, (1.0f / conv->frequency()) * 1e9), conv_index));
      }

    } else { // conv_queue is empty.
      // Short, so that a stop request is noticed quickly.
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  } // mutex scope
}

std::string Driver::minidump(const std::string & prefix)
{
  if (!log_enabled_) {
    throw std::runtime_error(
            "Minidump is off: set the minidump_duration parameter above 0 to keep recent data");
  }
  const std::filesystem::path folder = std::filesystem::current_path();
  if (helpers::filesystem::getBagsSize(folder) > helpers::filesystem::folderMaximumSize) {
    throw std::runtime_error(
            "The bags in " + folder.string() + " take more than 2 GB; move or delete some first");
  }

  // The minidump needs the bag: close a recording in progress.
  if (record_enabled_) {
    stopRecording();
  }

  // Stop bufferizing while the buffers are written, and resume afterwards.
  log_enabled_ = false;
  for (auto & [name, event] : event_map_) {
    event->isDumping(true);
  }
  const auto resume = [this] {
      for (auto & [name, event] : event_map_) {
        event->isDumping(false);
      }
      log_enabled_ = true;
    };

  std::lock_guard<std::mutex> lock_record(mutex_record_);
  try {
    recorder_->startRecord(prefix);
  } catch (...) {
    resume();
    throw;
  }
  const rclcpp::Time time = this->now();
  for (auto & [name, event] : event_map_) {
    event->writeDump(time);
  }
  for (auto & [name, recorder] : rec_map_) {
    recorder->writeDump(time);
  }
  resume();
  return recorder_->stopRecord();
}

void Driver::setBufferDuration(float duration)
{
  for (auto & [name, recorder] : rec_map_) {
    recorder->setBufferDuration(duration);
  }
  for (auto & [name, event] : event_map_) {
    event->setBufferDuration(duration);
  }
  buffer_duration_ = duration;
}

float Driver::getBufferDuration()
{
  return buffer_duration_;
}

void Driver::registerConverter(std::shared_ptr<converter::Converter> conv)
{
  std::lock_guard<std::mutex> lock(mutex_conv_queue_);
  int conv_index = converters_.size();
  converters_.push_back(conv);
  conv->reset();
  conv_queue_.push(ScheduledConverter(this->now(), conv_index));
}

void Driver::registerPublisher(
  const std::string & conv_name,
  std::shared_ptr<publisher::Publisher> pub)
{
  if (publish_enabled_) {
    pub->reset(this);
  }
  pub_map_.emplace(conv_name, pub);
}

void Driver::registerRecorder(
  const std::string & conv_name, std::shared_ptr<recorder::Recorder> rec,
  float frequency)
{
  rec->reset(recorder_, frequency);
  rec_map_.emplace(conv_name, rec);
}

void Driver::insertEventConverter(const std::string & key, std::shared_ptr<event::Event> event)
{
  event->resetRecorder(recorder_);
  event_map_.emplace(key, event);
}

void Driver::registerConverter(
  std::shared_ptr<converter::Converter> conv, std::shared_ptr<publisher::Publisher> pub,
  std::shared_ptr<recorder::Recorder> rec)
{
  registerConverter(conv);
  registerPublisher(conv->name(), pub);
  registerRecorder(conv->name(), rec, conv->frequency());
}

void Driver::registerPublisher(
  std::shared_ptr<converter::Converter> conv,
  std::shared_ptr<publisher::Publisher> pub)
{
  registerConverter(conv);
  registerPublisher(conv->name(), pub);
}

void Driver::registerRecorder(
  std::shared_ptr<converter::Converter> conv,
  std::shared_ptr<recorder::Recorder> rec)
{
  registerConverter(conv);
  registerRecorder(conv->name(), rec, conv->frequency());
}

bool Driver::registerMemoryConverter(
  const std::string & key, float frequency,
  const dataType::DataType & type)
{
  dataType::DataType data_type;
  qi::AnyValue value;
  try {
    qi::AnyObject p_memory = session_->service("ALMemory").value();
    value = p_memory.call<qi::AnyValue>("getData", key);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not get data in memory for the key: " << key);
    return false;
  }

  if (type == dataType::None) {
    try {
      data_type = helpers::naoqi::getDataType(value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Could not get a valid data type to register memory converter " << key <<
          "\nYou can enter it yourself, available types are:\n\t > 0 - None\n\t > 1 - Float\n\t > 2 - Int\n\t > 3 - String\n\t > 4 - Bool");
      return false;
    }
  } else {
    data_type = type;
  }

  switch (data_type) {
    case 0:
      return false;
      break;
    case 1:
      registerTypedMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::msg::FloatStamped>,
        recorder::BasicRecorder<naoqi_bridge_msgs::msg::FloatStamped>,
        converter::MemoryFloatConverter>(
        key, frequency);
      break;
    case 2:
      registerTypedMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::msg::IntStamped>,
        recorder::BasicRecorder<naoqi_bridge_msgs::msg::IntStamped>, converter::MemoryIntConverter>(
        key,
        frequency);
      break;
    case 3:
      registerTypedMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::msg::StringStamped>,
        recorder::BasicRecorder<naoqi_bridge_msgs::msg::StringStamped>,
        converter::MemoryStringConverter>(
        key, frequency);
      break;
    case 4:
      registerTypedMemoryConverter<publisher::BasicPublisher<naoqi_bridge_msgs::msg::BoolStamped>,
        recorder::BasicRecorder<naoqi_bridge_msgs::msg::BoolStamped>,
        converter::MemoryBoolConverter>(
        key,
        frequency);
      break;
    default:
      {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "Wrong data type. Available type are: \n\t > 0 - None\n\t > 1 - Float\n\t > 2 - Int\n\t > 3 - String\n\t > 4 - Bool");
        return false;
        break;
      }
  }
  return true;
}

void Driver::registerDefaultConverter()
{
  // init global tf2 buffer
  tf2_buffer_.reset<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock()) );
  tf2_buffer_->setUsingDedicatedThread(true);

  BootConfig config = boot_config_;
  // When depth comes from the stereo pair, the infrared camera is not available.
  if (has_stereo_) {
    config.ir_camera.enabled = false;
  }

  // Converters are scheduled in the order they are registered.
  registerStatusConverters(config);
  registerImuConverters(config);
  registerCameraConverters(config);
  registerMotionConverters(config);
  registerEventRegisters(config);
}

void Driver::registerStatusConverters(const BootConfig & config)
{
  // The info converter runs once (0 Hz): its publisher is latched and always counts as
  // subscribed, so later subscribers still get the robot information.
  if (config.info.enabled) {
    wireConverter(
      std::make_shared<converter::InfoConverter>("info", 0, session_),
      std::make_shared<publisher::InfoPublisher>("info"),
      std::make_shared<InfoRecorder>("info"));
  }

  // NAOqi logs go to /rosout only; they are not recorded.
  if (config.logs.enabled) {
    auto log_converter =
      std::make_shared<converter::LogConverter>("log", config.logs.frequency, session_);
    auto log_publisher = std::make_shared<publisher::LogPublisher>("/rosout");
    log_converter->registerCallback(
      message_actions::PUBLISH, [log_publisher](auto & msg) {log_publisher->publish(msg);});
    registerPublisher(log_converter, log_publisher);
  }

  if (config.diag.enabled) {
    wireConverter(
      std::make_shared<converter::DiagnosticsConverter>("diag", config.diag.frequency, session_),
      std::make_shared<DiagnosticsPublisher>("/diagnostics"),
      std::make_shared<recorder::DiagnosticsRecorder>("/diagnostics"));
  }
}

void Driver::registerImuConverters(const BootConfig & config)
{
  if (config.imu_torso.enabled) {
    wireConverter(
      std::make_shared<converter::ImuConverter>(
        "imu_torso", converter::IMU::TORSO, config.imu_torso.frequency, session_),
      std::make_shared<ImuPublisher>("imu/torso"),
      std::make_shared<ImuRecorder>("imu/torso"));
  }

  if (robot_ == robot::PEPPER && config.imu_base.enabled) {
    wireConverter(
      std::make_shared<converter::ImuConverter>(
        "imu_base", converter::IMU::BASE, config.imu_base.frequency, session_),
      std::make_shared<ImuPublisher>("imu/base"),
      std::make_shared<ImuRecorder>("imu/base"));
  }
}

void Driver::registerCameraConverters(const BootConfig & config)
{
  // Depth comes from the Xtion sensor or, on newer robots, from the stereo pair.
  const size_t depth_resolution = has_stereo_ ?
    config.depth_camera.stereo_resolution : config.depth_camera.xtion_resolution;

  if (config.front_camera.enabled) {
    wireConverter(
      std::make_shared<converter::CameraConverter>(
        "front_camera", config.front_camera.fps, session_, AL::kTopCamera,
        config.front_camera.resolution),
      std::make_shared<publisher::CameraPublisher>("camera/front/image_raw", AL::kTopCamera),
      std::make_shared<recorder::CameraRecorder>(
        "camera/front", config.front_camera.recorder_fps));
  }

  if (config.bottom_camera.enabled) {
    wireConverter(
      std::make_shared<converter::CameraConverter>(
        "bottom_camera", config.bottom_camera.fps, session_, AL::kBottomCamera,
        config.bottom_camera.resolution),
      std::make_shared<publisher::CameraPublisher>("camera/bottom/image_raw", AL::kBottomCamera),
      std::make_shared<recorder::CameraRecorder>(
        "camera/bottom", config.bottom_camera.recorder_fps));
  }

  if (robot_ == robot::PEPPER) {
    if (config.depth_camera.enabled) {
      wireConverter(
        std::make_shared<converter::CameraConverter>(
          "depth_camera", config.depth_camera.fps, session_, AL::kDepthCamera,
          depth_resolution, has_stereo_),
        std::make_shared<publisher::CameraPublisher>("camera/depth/image_raw", AL::kDepthCamera),
        std::make_shared<recorder::CameraRecorder>(
          "camera/depth", config.depth_camera.recorder_fps));
    }

    if (has_stereo_ && config.stereo_camera.enabled) {
      wireConverter(
        std::make_shared<converter::CameraConverter>(
          "stereo_camera", config.stereo_camera.fps, session_, AL::kInfraredOrStereoCamera,
          config.stereo_camera.resolution, has_stereo_),
        std::make_shared<publisher::CameraPublisher>(
          "camera/stereo/image_raw", AL::kInfraredOrStereoCamera),
        std::make_shared<recorder::CameraRecorder>(
          "camera/stereo", config.stereo_camera.recorder_fps));
    }

    if (config.ir_camera.enabled) {
      wireConverter(
        std::make_shared<converter::CameraConverter>(
          "infrared_camera", config.ir_camera.fps, session_, AL::kInfraredOrStereoCamera,
          config.ir_camera.resolution),
        std::make_shared<publisher::CameraPublisher>(
          "camera/ir/image_raw", AL::kInfraredOrStereoCamera),
        std::make_shared<recorder::CameraRecorder>("camera/ir", config.ir_camera.recorder_fps));
    }
  }
}

void Driver::registerMotionConverters(const BootConfig & config)
{
  if (config.joint_states.enabled) {
    wireConverter(
      std::make_shared<converter::JointStateConverter>(
        "joint_states", config.joint_states.frequency, tf2_buffer_, session_),
      std::make_shared<publisher::JointStatePublisher>("/joint_states"),
      std::make_shared<recorder::JointStateRecorder>("/joint_states"));
  }

  if (robot_ == robot::PEPPER && config.laser.enabled) {
    auto laser = std::make_shared<converter::LaserConverter>(
      "laser", config.laser.frequency, session_);
    laser->setLaserRanges(config.laser.range_min, config.laser.range_max);
    wireConverter(
      laser, std::make_shared<LaserPublisher>("laser"), std::make_shared<LaserRecorder>("laser"));
  }

  if (config.sonar.enabled) {
    const std::vector<std::string> sonar_topics = robot_ == robot::PEPPER ?
      std::vector<std::string>{"sonar/front", "sonar/back"} :
    std::vector<std::string>{"sonar/left", "sonar/right"};
    wireConverter(
      std::make_shared<converter::SonarConverter>("sonar", config.sonar.frequency, session_),
      std::make_shared<publisher::SonarPublisher>(sonar_topics),
      std::make_shared<recorder::SonarRecorder>(sonar_topics));
  }

  if (config.odom.enabled) {
    wireConverter(
      std::make_shared<converter::OdomConverter>("pepper_odom", config.odom.frequency, session_),
      std::make_shared<OdomPublisher>("pepper_odom"),
      std::make_shared<OdomRecorder>("pepper_odom"));
  }
}

void Driver::registerEventRegisters(const BootConfig & config)
{
  if (config.audio.enabled) {
    insertEventConverter("audio", makeAudioEventRegister("audio", 0, session_));
    if (keep_looping_) {
      try {
        event_map_.at("audio")->startProcess();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to start audio extraction: " << e.what());
        RCLCPP_WARN_STREAM(
          get_logger(),
          "Audio is being disabled automatically.\nTry specifying the --qi_listen_url option to an endpoint reachable by the robot to fix that.");
      }
    }
    if (publish_enabled_) {
      event_map_.at("audio")->isPublishing(true);
    }
  }

  if (config.bumper.enabled) {
    std::vector<std::string> bumper_events{"RightBumperPressed", "LeftBumperPressed"};
    if (robot_ == robot::PEPPER) {
      bumper_events.push_back("BackBumperPressed");
    }
    startEventRegister(
      "bumper", std::make_shared<BumperEventRegister>("bumper", bumper_events, 0, session_));
  }

  if (config.touch_hand.enabled) {
    const std::vector<std::string> hand_touch_events{
      "HandRightBackTouched", "HandRightLeftTouched", "HandRightRightTouched",
      "HandLeftBackTouched", "HandLeftLeftTouched", "HandLeftRightTouched"};
    startEventRegister(
      "hand_touch",
      std::make_shared<HandTouchEventRegister>("hand_touch", hand_touch_events, 0, session_));
  }

  if (config.touch_head.enabled) {
    const std::vector<std::string> head_touch_events{
      "FrontTactilTouched", "MiddleTactilTouched", "RearTactilTouched"};
    startEventRegister(
      "head_touch",
      std::make_shared<HeadTouchEventRegister>("head_touch", head_touch_events, 0, session_));
  }
}

void Driver::startEventRegister(const std::string & name, std::shared_ptr<event::Event> event)
{
  insertEventConverter(name, event);
  if (keep_looping_) {
    event_map_.at(name)->startProcess();
  }
  if (publish_enabled_) {
    event_map_.at(name)->isPublishing(true);
  }
}

// public interface here
void Driver::registerSubscriber(std::shared_ptr<subscriber::Subscriber> sub)
{
  const auto it = std::find_if(
    subscribers_.begin(), subscribers_.end(),
    [&sub](const auto & existing) {return existing->name() == sub->name();});
  if (it == subscribers_.end()) {
    subscribers_.push_back(sub);
    RCLCPP_INFO_STREAM(get_logger(), "registered subscriber:\t" << sub->name());
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "re-initialized existing subscriber:\t" << (*it)->name());
  }
}

void Driver::registerDefaultSubscriber()
{
  if (!subscribers_.empty()) {
    return;
  }
  registerSubscriber(
    std::make_shared<naoqi::subscriber::TeleopSubscriber>(
      "teleop", "/cmd_vel",
      "/joint_angles", "/joint_angles_trajectory", session_) );
  registerSubscriber(
    std::make_shared<naoqi::subscriber::MovetoSubscriber>(
      "moveto",
      "/move_base_simple/goal", session_, tf2_buffer_) );
  registerSubscriber(
    std::make_shared<naoqi::subscriber::SpeechSubscriber>(
      "speech", "/speech",
      session_) );
}

void Driver::registerService(std::shared_ptr<service::Service> srv)
{
  services_.push_back(srv);
}

void Driver::registerDefaultServices()
{
  registerService(
    std::make_shared<service::TriggerService>(
      "start_recording", "/naoqi_driver/start_recording", [this] {return startRecording();}));
  registerService(
    std::make_shared<service::TriggerService>(
      "stop_recording", "/naoqi_driver/stop_recording", [this] {return stopRecording();}));
  registerService(
    std::make_shared<service::TriggerService>(
      "minidump", "/naoqi_driver/minidump", [this] {return minidump("naoqi_driver_minidump");}));
  registerService(
    std::make_shared<service::RobotConfigService>(
      "get_robot_config",
      "/naoqi_driver/get_robot_config", session_) );
  registerService(
    std::make_shared<service::SetLanguageService>(
      "set_language",
      "/naoqi_driver/set_language", session_) );
  registerService(
    std::make_shared<service::GetLanguageService>(
      "get_language",
      "/naoqi_driver/get_language", session_) );
  registerService(
    std::make_shared<service::OdomResetService>(
      "reset_odometry",
      "/naoqi_driver/reset_odometry", session_) );
}

std::vector<std::string> Driver::getAvailableConverters()
{
  std::vector<std::string> conv_list;
  for (const auto & conv : converters_) {
    conv_list.push_back(conv->name());
  }
  for (const auto & [name, event] : event_map_) {
    conv_list.push_back(name);
  }

  return conv_list;
}

void Driver::startPublishing()
{
  publish_enabled_ = true;
  for (auto & [name, event] : event_map_) {
    event->isPublishing(true);
  }
}

void Driver::stopPublishing()
{
  publish_enabled_ = false;
  for (auto & [name, event] : event_map_) {
    event->isPublishing(false);
  }
}

std::vector<std::string> Driver::getSubscribedPublishers() const
{
  std::vector<std::string> publisher;
  for (const auto & [name, pub] : pub_map_) {
    if (pub->isSubscribed()) {
      publisher.push_back(pub->topic());
    }
  }
  return publisher;
}

std::string Driver::startRecording()
{
  std::lock_guard<std::mutex> lock_record(mutex_record_);
  const std::string bag = recorder_->startRecord();
  for (auto & [name, recorder] : rec_map_) {
    recorder->subscribe(true);
  }
  for (auto & [name, event] : event_map_) {
    event->isRecording(true);
  }
  record_enabled_ = true;
  RCLCPP_INFO(
    get_logger(), "Recording %zu sensors into %s", rec_map_.size() + event_map_.size(),
    bag.c_str());
  return bag;
}

std::string Driver::stopRecording()
{
  std::lock_guard<std::mutex> lock_record(mutex_record_);
  record_enabled_ = false;
  for (auto & [name, recorder] : rec_map_) {
    recorder->subscribe(false);
  }
  for (auto & [name, event] : event_map_) {
    event->isRecording(false);
  }
  return recorder_->stopRecord();
}

void Driver::startLogging()
{
  log_enabled_ = true;
}

void Driver::stopLogging()
{
  log_enabled_ = false;
}

void Driver::stop()
{
  RCLCPP_INFO(get_logger(), "naoqi_driver is shutting down");
  keep_looping_ = false;

  // ALMotion.move() keeps the last velocity, so leaving without stopping would
  // let the base drive on. Each call is independent: one failing must not skip the others.
  if (session_) {
    const std::vector<std::pair<std::string, std::string>> halts = {
      {"ALMotion", "stopMove"},
      {"ALAnimatedSpeech", "stopAll"},
      {"ALTextToSpeech", "stopAll"},
      {"ALAudioPlayer", "stopAll"},
    };
    for (const auto & halt : halts) {
      try {
        session_->service(halt.first).value().call<void>(halt.second);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "%s.%s failed on shutdown: %s",
          halt.first.c_str(), halt.second.c_str(), e.what());
      }
    }
  }

  for (auto & [name, event] : event_map_) {
    event->stopProcess();
  }

  // Close a bag still being recorded, so that it is complete (metadata.yaml).
  if (record_enabled_) {
    try {
      RCLCPP_INFO(get_logger(), "Saved the recording in %s", stopRecording().c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Could not close the recording: %s", e.what());
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_conv_queue_);
    converters_.clear();
    subscribers_.clear();
    event_map_.clear();
  }
  // No spin_some() here: a /cmd_vel still queued would run after stopMove and could
  // start the base again.
}

void Driver::parseJsonFile(std::string filepath, boost::property_tree::ptree & pt)
{
  // Open json file and parse it
  std::ifstream json_file;
  json_file.open(filepath.c_str(), std::ios_base::in);
  if (!json_file.is_open()) {
    throw std::runtime_error("Failed to open JSON file: " + filepath);
  }
  boost::property_tree::json_parser::read_json(json_file, pt);
  json_file.close();
}

void Driver::addMemoryConverters(std::string filepath)
{
  // Open the file filepath and parse it
  boost::property_tree::ptree pt;
  parseJsonFile(filepath, pt);

  // Get the frequency requested (default to 10 Hz)
  float frequency = 10.0f;
  try {
    frequency = pt.get<float>("frequency");
  } catch (const boost::property_tree::ptree_bad_data & e) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "\"frequency\" could not be interpreted as float: " << e.what());
    RCLCPP_INFO_STREAM(get_logger(), "Default to 10 Hz");
  } catch (const boost::property_tree::ptree_bad_path & e) {
    RCLCPP_WARN_STREAM(get_logger(), "\"frequency\" was not found: " << e.what());
    RCLCPP_INFO_STREAM(get_logger(), "Default to 10 Hz");
  }

  // Get the topic name requested
  std::string topic;
  try {
    topic = pt.get<std::string>("topic");
  } catch (const boost::property_tree::ptree_error & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "\"topic\" could not be retrieved: " << e.what() << "\nCannot add new converters");
    return;
  }

  std::vector<std::string> list;
  try {
    for (boost::property_tree::ptree::value_type & v: pt.get_child("memKeys")) {
      std::string topic = v.second.get_value<std::string>();
      list.push_back(topic);
    }
  } catch (const boost::property_tree::ptree_error & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "A problem occured during the reading of the mem keys list: " << e.what() <<
        "\nCannot add new converters");
    return;
  }

  if (list.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "The list of keys to add is empty. ");
    return;
  }

  wireConverter(
    std::make_shared<converter::MemoryListConverter>(list, topic, frequency, session_),
    std::make_shared<MemoryListPublisher>(topic),
    std::make_shared<MemoryListRecorder>(topic));
}

bool Driver::registerEventConverter(const std::string & key, const dataType::DataType & type)
{
  dataType::DataType data_type;
  qi::AnyValue value;
  try {
    qi::AnyObject p_memory = session_->service("ALMemory").value();
    value = p_memory.call<qi::AnyValue>("getData", key);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not get data in memory for the key: " << key);
    return false;
  }

  if (type == dataType::None) {
    try {
      data_type = helpers::naoqi::getDataType(value);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Could not get a valid data type to register memory converter " << key <<
          "\nYou can enter it yourself, available types are:\n\t > 0 - None\n\t > 1 - Float\n\t > 2 - Int\n\t > 3 - String\n\t > 4 - Bool");
      return false;
    }
  } else {
    data_type = type;
  }

  switch (data_type) {
    case 0:
      return false;
      break;
    case 1:
      {
        std::shared_ptr<EventRegister<converter::MemoryFloatConverter,
          publisher::BasicPublisher<naoqi_bridge_msgs::msg::FloatStamped>,
          recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::FloatStamped>>> event_register =
          std::make_shared<EventRegister<converter::MemoryFloatConverter,
            publisher::BasicPublisher<naoqi_bridge_msgs::msg::FloatStamped>,
            recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::FloatStamped>>>(key, session_);
        insertEventConverter(key, event_register);
        break;
      }
    case 2:
      {
        std::shared_ptr<EventRegister<converter::MemoryIntConverter,
          publisher::BasicPublisher<naoqi_bridge_msgs::msg::IntStamped>,
          recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::IntStamped>>> event_register =
          std::make_shared<EventRegister<converter::MemoryIntConverter,
            publisher::BasicPublisher<naoqi_bridge_msgs::msg::IntStamped>,
            recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::IntStamped>>>(key, session_);
        insertEventConverter(key, event_register);
        break;
      }
    case 3:
      {
        std::shared_ptr<EventRegister<converter::MemoryStringConverter,
          publisher::BasicPublisher<naoqi_bridge_msgs::msg::StringStamped>,
          recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::StringStamped>>> event_register =
          std::make_shared<EventRegister<converter::MemoryStringConverter,
            publisher::BasicPublisher<naoqi_bridge_msgs::msg::StringStamped>,
            recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::StringStamped>>>(key, session_);
        insertEventConverter(key, event_register);
        break;
      }
    case 4:
      {
        std::shared_ptr<EventRegister<converter::MemoryBoolConverter,
          publisher::BasicPublisher<naoqi_bridge_msgs::msg::BoolStamped>,
          recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::BoolStamped>>> event_register =
          std::make_shared<EventRegister<converter::MemoryBoolConverter,
            publisher::BasicPublisher<naoqi_bridge_msgs::msg::BoolStamped>,
            recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::BoolStamped>>>(key, session_);
        insertEventConverter(key, event_register);
        break;
      }
    default:
      {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "Wrong data type. Available type are: \n\t > 0 - None\n\t > 1 - Float\n\t > 2 - Int\n\t > 3 - String\n\t > 4 - Bool");
        return false;
      }
  }

  if (keep_looping_) {
    event_map_.at(key)->startProcess();
  }
  if (publish_enabled_) {
    event_map_.at(key)->isPublishing(true);
  }

  return true;
}

}  // namespace naoqi
