// Recording into rosbag2 bags: what GlobalRecorder writes, read back with
// rosbag2_cpp::Reader; the driver's start/stop/minidump; the Trigger services.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <naoqi_driver/naoqi_driver.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

#include "helpers/filesystem_helpers.hpp"
#include "recorder/basic.hpp"
#include "services/trigger.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using naoqi::recorder::GlobalRecorder;
using Imu = sensor_msgs::msg::Imu;

namespace
{

/** A message read from a bag. It owns a copy of the bytes: the reader's buffers are
 * freed by the storage plugin, which is unloaded with the reader. */
struct BagMessage
{
  std::string topic;
  std::string type;
  int64_t time_stamp;
  std::vector<uint8_t> data;
};

/** Every message of the bag in path, in time order, with the type of its topic. */
std::vector<BagMessage> readBag(const std::string & path)
{
  rosbag2_storage::StorageOptions storage;
  storage.uri = path;
  storage.storage_id = "sqlite3";
  rosbag2_cpp::Reader reader;
  reader.open(storage);
  std::map<std::string, std::string> types;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    types[topic.name] = topic.type;
  }
  std::vector<BagMessage> messages;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const auto & bytes = *message->serialized_data;
    messages.push_back(
      {message->topic_name, types[message->topic_name], message->time_stamp,
        std::vector<uint8_t>(bytes.buffer, bytes.buffer + bytes.buffer_length)});
  }
  return messages;
}

template<typename T>
T deserialize(const BagMessage & message)
{
  rclcpp::SerializedMessage serialized(message.data.size());
  auto & bytes = serialized.get_rcl_serialized_message();
  std::copy(message.data.begin(), message.data.end(), bytes.buffer);
  bytes.buffer_length = message.data.size();
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &msg);
  return msg;
}

/** An IMU message carrying x, stamped at x seconds (0: unstamped). */
Imu imu(double x)
{
  Imu msg;
  msg.linear_acceleration.x = x;
  msg.header.stamp.sec = static_cast<int32_t>(x);
  return msg;
}

/** An IMU message carrying x, stamped age seconds ago (the minidump keeps recent ones). */
Imu recentImu(double x, double age)
{
  Imu msg = imu(x);
  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now() - rclcpp::Duration::from_seconds(age);
  return msg;
}

}  // namespace

class RecordingTest : public naoqi_test::RosTest {};

TEST_F(RecordingTest, BagHoldsWhatWasWrittenUnderTheLiveTopicNames)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  GlobalRecorder global("naoqi_driver");
  global.write("imu/torso", imu(1));  // not recording: dropped

  const std::string bag = global.startRecord("unit");
  EXPECT_EQ(bag.rfind((dir.path() / "unit_").string(), 0), 0u) << bag;
  global.write("imu/torso", imu(12), imu(12).header.stamp);
  std::vector<geometry_msgs::msg::TransformStamped> tf(1);
  tf[0].header.stamp.sec = 13;
  tf[0].child_frame_id = "Head";
  global.write("/tf", tf);
  EXPECT_EQ(global.stopRecord(), bag);
  global.write("imu/torso", imu(14));  // stopped: dropped

  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(bag) / "metadata.yaml"));
  const auto messages = readBag(bag);
  ASSERT_EQ(messages.size(), 2u);
  EXPECT_EQ(messages[0].topic, "/naoqi_driver/imu/torso");
  EXPECT_EQ(messages[0].type, "sensor_msgs/msg/Imu");
  EXPECT_EQ(messages[0].time_stamp, 12'000'000'000);
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages[0]).linear_acceleration.x, 12.0);
  EXPECT_EQ(messages[1].topic, "/tf");
  EXPECT_EQ(messages[1].type, "tf2_msgs/msg/TFMessage");
  EXPECT_EQ(messages[1].time_stamp, 13'000'000'000);
  EXPECT_EQ(
    deserialize<tf2_msgs::msg::TFMessage>(messages[1]).transforms[0].child_frame_id,
    "Head");
}

TEST_F(RecordingTest, RootNamespaceRecordsRelativeTopicsAtTheRoot)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  for (const std::string ns : {"/", ""}) {
    GlobalRecorder global(ns);
    const std::string bag = global.startRecord("unit");
    global.write("imu/torso", imu(1), imu(1).header.stamp);
    global.stopRecord();
    const auto messages = readBag(bag);
    ASSERT_EQ(messages.size(), 1u);
    EXPECT_EQ(messages[0].topic, "/imu/torso") << "namespace '" << ns << "'";
  }
}

TEST_F(RecordingTest, OneBagAtATimeAndNamesNeverCollide)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  GlobalRecorder global("naoqi_driver");
  const std::string first = global.startRecord("unit");
  EXPECT_THROW(global.startRecord("unit"), std::runtime_error);
  global.stopRecord();
  EXPECT_THROW(global.stopRecord(), std::runtime_error);

  // Within the same second: a numbered name rather than a failure.
  const std::string second = global.startRecord("unit");
  global.stopRecord();
  EXPECT_NE(first, second);
}

TEST_F(RecordingTest, DriverRecordsItsRecordersIntoABag)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  auto driver = std::make_shared<naoqi::Driver>();
  auto recorder = std::make_shared<naoqi::recorder::BasicRecorder<Imu>>("imu");
  driver->registerRecorder("imu", recorder, 10.f);

  const std::string bag = driver->startRecording();
  EXPECT_TRUE(recorder->isSubscribed());
  EXPECT_THROW(driver->startRecording(), std::runtime_error);
  recorder->write(imu(3));  // what the converter's RECORD action calls
  EXPECT_EQ(driver->stopRecording(), bag);
  EXPECT_FALSE(recorder->isSubscribed());
  EXPECT_THROW(driver->stopRecording(), std::runtime_error);

  const auto messages = readBag(bag);
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_EQ(messages[0].topic, "/imu");  // the driver node runs in the root namespace
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages[0]).linear_acceleration.x, 3.0);
}

TEST_F(RecordingTest, StoppingTheDriverClosesTheBag)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  auto driver = std::make_shared<naoqi::Driver>();
  auto recorder = std::make_shared<naoqi::recorder::BasicRecorder<Imu>>("imu");
  driver->registerRecorder("imu", recorder, 10.f);
  const std::string bag = driver->startRecording();
  recorder->write(imu(3));

  driver->stop();
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(bag) / "metadata.yaml"));
  EXPECT_EQ(readBag(bag).size(), 1u);
}

TEST_F(RecordingTest, MinidumpWritesTheNewestBufferedMessages)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  auto driver = std::make_shared<naoqi::Driver>();
  auto recorder = std::make_shared<naoqi::recorder::BasicRecorder<Imu>>("imu");
  driver->registerRecorder("imu", recorder, 10.f);
  driver->setBufferDuration(1.f);  // 10 messages at 10 Hz
  driver->startLogging();
  for (int i = 1; i <= 15; ++i) {
    recorder->bufferize(recentImu(i, 0.01 * (15 - i)));
  }

  const std::string bag = driver->minidump("unit");
  const auto messages = readBag(bag);
  ASSERT_EQ(messages.size(), 10u);
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages.front()).linear_acceleration.x, 6.0);
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages.back()).linear_acceleration.x, 15.0);

  // Buffering resumed after the dump.
  recorder->bufferize(recentImu(16, 0.0));
  const auto again = readBag(driver->minidump("unit"));
  EXPECT_DOUBLE_EQ(deserialize<Imu>(again.back()).linear_acceleration.x, 16.0);
}

TEST_F(RecordingTest, MinidumpLeavesOutMessagesOlderThanTheBufferDuration)
{
  // The buffer holds 10 messages, sized for 10 Hz; a slower sensor fills it with older
  // data than the buffer duration, which the dump must not include.
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  auto driver = std::make_shared<naoqi::Driver>();
  auto recorder = std::make_shared<naoqi::recorder::BasicRecorder<Imu>>("imu");
  driver->registerRecorder("imu", recorder, 10.f);
  driver->setBufferDuration(1.f);
  driver->startLogging();
  recorder->bufferize(recentImu(1, 5.0));
  recorder->bufferize(recentImu(2, 2.0));
  recorder->bufferize(recentImu(3, 0.5));
  recorder->bufferize(recentImu(4, 0.0));

  const auto messages = readBag(driver->minidump("unit"));
  ASSERT_EQ(messages.size(), 2u);
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages.front()).linear_acceleration.x, 3.0);
  EXPECT_DOUBLE_EQ(deserialize<Imu>(messages.back()).linear_acceleration.x, 4.0);
}

TEST_F(RecordingTest, MinidumpRefusesWhenTheBagsTakeTooMuchSpace)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  const auto old_bag = dir.path() / "old" / "old_0.db3";
  std::filesystem::create_directories(old_bag.parent_path());
  std::ofstream(old_bag).put('x');
  // Sparse: reports the size without using the disk.
  std::filesystem::resize_file(old_bag, naoqi::helpers::filesystem::folderMaximumSize + 1);

  auto driver = std::make_shared<naoqi::Driver>();
  driver->startLogging();
  try {
    driver->minidump("unit");
    FAIL() << "minidump must refuse above the size limit";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("2 GB"), std::string::npos) << e.what();
  }
  EXPECT_EQ(naoqi::helpers::filesystem::getBags(dir.path()).size(), 1u);
}

TEST_F(RecordingTest, TriggerServiceAnswersWithTheResultOrTheError)
{
  bool fail = false;
  naoqi::service::TriggerService service(
    "unit", "/test_trigger", [&fail]() -> std::string {
      if (fail) {
        throw std::runtime_error("disk full");
      }
      return "/tmp/bag";
    });
  service.reset(node_.get());
  auto client = node_->create_client<std_srvs::srv::Trigger>("/test_trigger");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  ASSERT_TRUE(client->wait_for_service(2s));

  const auto call = [&] {
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      EXPECT_EQ(executor.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
      return future.get();
    };
  const auto ok = call();
  EXPECT_TRUE(ok->success);
  EXPECT_EQ(ok->message, "/tmp/bag");
  fail = true;
  const auto error = call();
  EXPECT_FALSE(error->success);
  EXPECT_EQ(error->message, "disk full");
  executor.remove_node(node_);
}
