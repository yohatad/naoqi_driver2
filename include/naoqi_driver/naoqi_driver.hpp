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

#ifndef NAOQI_DRIVER_HPP
#define NAOQI_DRIVER_HPP

#include <memory>
#include <functional>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>

/*
* BOOST
*/
#include <boost/property_tree/ptree.hpp>

/*
* ALDEB
*/
#include <qi/session.hpp>

/*
* PUBLIC INTERFACE
*/
#include <naoqi_driver/boot_config.hpp>
#include <naoqi_driver/converter/converter.hpp>
#include <naoqi_driver/converter/route_actions.hpp>
#include <naoqi_driver/publisher/publisher.hpp>
#include <naoqi_driver/subscriber/subscriber.hpp>
#include <naoqi_driver/service/service.hpp>
#include <naoqi_driver/recorder/recorder.hpp>
#include <naoqi_driver/event/event.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

namespace tf2_ros
{
class Buffer;
}  // namespace tf2_ros

namespace naoqi
{

namespace recorder
{
class GlobalRecorder;
}  // namespace recorder
/**
* @brief Interface for naoqi driver which is registered as a naoqi2 Module,
* once the external roscore ip is set, this class will advertise and publish ros messages
*/
class Driver : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the naoqi driver
   *
   */
  Driver();

  /**
  * @brief Destructor for naoqi driver,
  * destroys all ros nodehandle and shutsdown all publisher
  */
  ~Driver() override;

  /**
   * @brief Register the ROS components and start the ROS loop.
   */
  void run();

  /**
   * @brief Set the Session object
   *
   */
  void setQiSession(const qi::SessionPtr & session_ptr);

  void stop();

  /**
   * @brief Async-signal-safe request to break out of the run() loop.
   * Call this from a signal handler; call stop() afterwards from
   * normal (non-signal) context to perform the actual cleanup.
   */
  void requestStop()
  {
    keep_looping_ = false;
  }

  /**
   * @brief Writes the buffered recent data (see startLogging()) into a new bag
   * @return the bag folder
   * @throw std::runtime_error if logging is off, the bags take too much space, or
   * the bag cannot be written
   */
  std::string minidump(const std::string & prefix);

  void setBufferDuration(float duration);
  float getBufferDuration();

  /**
   * @brief registers generall converter units
   * they are connected via callbacks to various actions such as record, log, publish
   */
  void registerConverter(std::shared_ptr<converter::Converter> conv);

  /**
   * @brief prepare and register a publisher
   * @param conv_name the name of the converter related to the publisher
   * @param pub       the publisher to add
   */
  void registerPublisher(const std::string & conv_name, std::shared_ptr<publisher::Publisher> pub);

  /**
   * @brief prepare and register a recorder
   * @param conv_name the name of the converter related to the recorder
   * @param rec       the recorder to add
   */
  void registerRecorder(
    const std::string & conv_name, std::shared_ptr<recorder::Recorder> rec,
    float frequency);

  /**
   * @brief register a converter with an associated publisher and recorder
   */
  void registerConverter(
    std::shared_ptr<converter::Converter> conv, std::shared_ptr<publisher::Publisher> pub,
    std::shared_ptr<recorder::Recorder> rec);

  /**
   * @brief register a converter with an associated publisher instance
   */
  void registerPublisher(
    std::shared_ptr<converter::Converter> conv,
    std::shared_ptr<publisher::Publisher> pub);

  /**
   * @brief register a converter with an associated recorder instance
   */
  void registerRecorder(
    std::shared_ptr<converter::Converter> conv,
    std::shared_ptr<recorder::Recorder> rec);

  /**
   * @brief Registers a converter for a given memory key
   */
  bool registerMemoryConverter(
    const std::string & key, float frequency,
    const dataType::DataType & type);

  /**
   * @brief Registers a converter for a given memory event
   */
  bool registerEventConverter(const std::string & key, const dataType::DataType & type);

  /**
   * @brief get all available converters
   */
  std::vector<std::string> getAvailableConverters();

  /**
   * @brief get all subscribed publishers
   */
  std::vector<std::string> getSubscribedPublishers() const;

  std::string _whoIsYourDaddy()
  {
    return "A sugar bear";
  }

  /**
  * @brief registers a subscriber
  * @param subscriber to register
  * @see Subscriber
  * @note it will be called by value to expose that internally there will be a copy,
  * eventually this should be replaced by move semantics C++11
  */
  void registerSubscriber(std::shared_ptr<subscriber::Subscriber> sub);

  /**
   * @brief registers a service
   * @param service to register
   * @see Service
   * @note it will be called by value to expose that internally there will be a copy,
   * eventually this should be replaced by move semantics C++11
   */
  void registerService(std::shared_ptr<service::Service> srv);

  /**
  * @brief Starts publishing with all registered publishers
  */
  void startPublishing();

  /**
  * @brief Stops publishing with all registered publishers
  */
  void stopPublishing();

  /**
  * @brief Starts recording every registered sensor into a new bag
  * @return the bag folder
  * @throw std::runtime_error if already recording or the bag cannot be created
  */
  std::string startRecording();

  /**
  * @brief Stops recording and closes the bag
  * @return the bag folder
  * @throw std::runtime_error if not recording
  */
  std::string stopRecording();

  /** @brief Starts keeping the recent data of every sensor in memory, for minidump() */
  void startLogging();

  void stopLogging();

  /**
   * @brief Adds converters for the memory keys listed in a JSON file
   */
  void addMemoryConverters(std::string filepath);

  void parseJsonFile(std::string filepath, boost::property_tree::ptree & pt);

private:
  qi::SessionPtr session_;
  robot::Robot robot_;

  std::atomic<bool> publish_enabled_;
  std::atomic<bool> record_enabled_;
  std::atomic<bool> log_enabled_;
  std::atomic<bool> keep_looping_;
  bool has_stereo_;

  const size_t freq_;

  std::shared_ptr<recorder::GlobalRecorder> recorder_;

  /* boot config */
  BootConfig boot_config_;
  void loadBootConfig();

  void registerDefaultConverter();
  // Groups called by registerDefaultConverter(), in registration order.
  void registerStatusConverters(const BootConfig & config);  // info, NAOqi logs, diagnostics
  void registerImuConverters(const BootConfig & config);
  void registerCameraConverters(const BootConfig & config);
  void registerMotionConverters(const BootConfig & config);  // joint states, laser, sonar, odom
  void registerEventRegisters(const BootConfig & config);  // audio, bumpers, hand and head touch
  /** Adds an event register, starting it and enabling publishing if the driver already is. */
  void startEventRegister(const std::string & name, std::shared_ptr<event::Event> event);
  void registerDefaultSubscriber();
  void registerDefaultServices();
  void insertEventConverter(const std::string & key, std::shared_ptr<event::Event> event);

  /** Routes the converter's actions to the publisher and recorder, then registers them. */
  template<typename Converter, typename Publisher, typename Recorder>
  void wireConverter(
    std::shared_ptr<Converter> converter, std::shared_ptr<Publisher> publisher,
    std::shared_ptr<Recorder> recorder)
  {
    converter::routeActions(*converter, publisher, recorder);
    registerConverter(converter, publisher, recorder);
  }

  template<typename Publisher, typename Recorder, typename Converter>
  void registerTypedMemoryConverter(const std::string & key, float frequency)
  {
    wireConverter(
      std::make_shared<Converter>(key, frequency, session_, key),
      std::make_shared<Publisher>(key),
      std::make_shared<Recorder>(key));
  }

  void rosIteration();

  std::mutex mutex_conv_queue_;
  std::mutex mutex_record_;

  std::vector<std::shared_ptr<converter::Converter>> converters_;
  std::map<std::string, std::shared_ptr<publisher::Publisher>> pub_map_;
  std::map<std::string, std::shared_ptr<recorder::Recorder>> rec_map_;
  std::map<std::string, std::shared_ptr<event::Event>> event_map_;

  std::vector<std::shared_ptr<subscriber::Subscriber>> subscribers_;
  std::vector<std::shared_ptr<service::Service>> services_;

  float buffer_duration_;

  /** Pub Publisher to execute at a specific time */
  struct ScheduledConverter
  {
    ScheduledConverter(const rclcpp::Time & schedule, size_t conv_index)
    : schedule_(schedule), conv_index_(conv_index)
    {
    }

    bool operator<(const ScheduledConverter & sp_in) const
    {
      return schedule_ > sp_in.schedule_;
    }
    /** Time at which the publisher will be called */
    rclcpp::Time schedule_;
    /** Time at which the publisher will be called */
    size_t conv_index_;
  };

  /** Priority queue to process the publishers according to their frequency */
  std::priority_queue<ScheduledConverter> conv_queue_;

  /** tf2 buffer that will be shared between different publishers/subscribers
   * This is only for performance improvements
   */
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
};

}  // namespace naoqi

#endif
