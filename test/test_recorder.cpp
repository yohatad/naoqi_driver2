// Recorders moved from boost::mutex / boost::shared_ptr to std::mutex / std::shared_ptr.
// These tests pin the buffering behaviour and hammer the locked paths from several
// threads (run them under -fsanitize=thread to turn races into hard failures).

#include <atomic>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/imu.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>

#include "recorder/basic.hpp"
#include "test_utils.hpp"

using naoqi::recorder::GlobalRecorder;
using Imu = sensor_msgs::msg::Imu;

namespace
{

/** Exposes the protected ring buffer so its contents can be checked. */
class ProbeRecorder : public naoqi::recorder::BasicRecorder<Imu>
{
public:
  using BasicRecorder::BasicRecorder;
  std::size_t size() const {return buffer_.size();}
  std::size_t capacity() const {return buffer_.capacity();}
  double frontX() const {return buffer_.front().linear_acceleration.x;}
  double backX() const {return buffer_.back().linear_acceleration.x;}
  double xAt(std::size_t i) const {return buffer_[i].linear_acceleration.x;}
};

Imu imuWithX(double x)
{
  Imu msg;
  msg.linear_acceleration.x = x;
  return msg;
}

}  // namespace

class RecorderTest : public naoqi_test::RosTest {};

TEST_F(RecorderTest, ResetSizesBufferFromConverterFrequency)
{
  ProbeRecorder rec("imu");
  rec.reset(std::make_shared<GlobalRecorder>("naoqi_driver"), 10.f);

  EXPECT_TRUE(rec.isInitialized());
  // bufferDefaultDuration (10 s) * 10 Hz
  EXPECT_EQ(rec.capacity(), 100u);
}

TEST_F(RecorderTest, BufferizeKeepsOnlyTheNewestMessages)
{
  ProbeRecorder rec("imu");
  rec.reset(std::make_shared<GlobalRecorder>("naoqi_driver"), 1.f);  // 10 slots
  for (int i = 0; i < 25; ++i) {
    rec.bufferize(imuWithX(i));
  }

  ASSERT_EQ(rec.size(), 10u);
  EXPECT_DOUBLE_EQ(rec.frontX(), 15.0);
  EXPECT_DOUBLE_EQ(rec.backX(), 24.0);
}

TEST_F(RecorderTest, BufferFrequencyDownsamples)
{
  // Converter at 10 Hz, buffer at 5 Hz: every second message is kept, in
  // 10 s * 10 Hz / 2 = 50 slots. The ring starts empty: a minidump right after
  // start-up must not write default-constructed messages.
  ProbeRecorder rec("imu", 5.f);
  rec.reset(std::make_shared<GlobalRecorder>("naoqi_driver"), 10.f);
  EXPECT_EQ(rec.size(), 0u);
  EXPECT_EQ(rec.capacity(), 50u);
  for (int i = 1; i <= 6; ++i) {
    rec.bufferize(imuWithX(i));
  }

  ASSERT_EQ(rec.size(), 3u);
  EXPECT_DOUBLE_EQ(rec.xAt(0), 2.0);
  EXPECT_DOUBLE_EQ(rec.xAt(1), 4.0);
  EXPECT_DOUBLE_EQ(rec.xAt(2), 6.0);
}

TEST_F(RecorderTest, SetBufferDurationResizesCapacity)
{
  ProbeRecorder rec("imu");
  rec.reset(std::make_shared<GlobalRecorder>("naoqi_driver"), 10.f);
  rec.setBufferDuration(2.f);
  EXPECT_EQ(rec.capacity(), 20u);
}

TEST_F(RecorderTest, ConcurrentBufferizeResizeAndDumpAreSafe)
{
  ProbeRecorder rec("imu");
  auto global = std::make_shared<GlobalRecorder>("naoqi_driver");
  rec.reset(global, 100.f);

  std::atomic<bool> stop{false};
  std::vector<std::thread> producers;
  // 3000, not 20000. The dumper and resizer below spin unbounded until the
  // producers finish, and writeDump() is O(buffer), so the work is set by how
  // long the producers run -- which means spare cores make this test SLOWER,
  // not faster. At 20000 it took 87 s on 28 idle cores against a 120 s timeout
  // for the whole binary, and overran it on CI, where ctest killing the binary
  // reports as "the test did not generate a result file" rather than as a
  // timeout. Races are found by conflicting access pairs, not by volume:
  // 6 threads x 3000 still wraps the buffer hundreds of times under the
  // resizer, and it keeps the suite at ~27 s instead of ~95 s.
  for (int t = 0; t < 6; ++t) {
    producers.emplace_back(
      [&rec, t] {
        for (int i = 0; i < 3000; ++i) {
          rec.bufferize(imuWithX(t * 100000 + i));
        }
      });
  }
  std::thread resizer([&] {
      int i = 0;
      while (!stop) {
        rec.setBufferDuration(1.f + static_cast<float>(i++ % 5));
      }
    });
  std::thread dumper([&] {
      const auto now = node_->now();
      while (!stop) {
        rec.writeDump(now);
      }
    });

  for (auto & p : producers) {
    p.join();
  }
  stop = true;
  resizer.join();
  dumper.join();

  EXPECT_LE(rec.size(), rec.capacity());
}

TEST_F(RecorderTest, GlobalRecorderCreatesBagFolderInWorkingDirectory)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  GlobalRecorder global("naoqi_driver");

  EXPECT_FALSE(global.isStarted());
  const std::string opened = global.startRecord("unit");
  EXPECT_TRUE(global.isStarted());

  const std::string bag = global.stopRecord();
  EXPECT_FALSE(global.isStarted());
  EXPECT_EQ(bag, opened);

  const std::string expected_prefix = std::filesystem::current_path().string() + "/unit_";
  EXPECT_EQ(bag.rfind(expected_prefix, 0), 0u) << bag;
  EXPECT_TRUE(std::filesystem::is_directory(bag)) << bag;
  EXPECT_TRUE(std::filesystem::exists(std::filesystem::path(bag) / "metadata.yaml"));
}

TEST_F(RecorderTest, GlobalRecorderStopWithoutStartIsRejected)
{
  GlobalRecorder global("naoqi_driver");
  EXPECT_THROW(global.stopRecord(), std::runtime_error);
}

TEST_F(RecorderTest, GlobalRecorderConcurrentStartStopAndWriteDoNotDeadlock)
{
  naoqi_test::TempDir dir(/*chdir_into=*/ true);
  GlobalRecorder global("naoqi_driver");

  std::atomic<bool> stop{false};
  std::vector<std::thread> writers;
  for (int t = 0; t < 4; ++t) {
    writers.emplace_back(
      [&] {
        std::vector<geometry_msgs::msg::TransformStamped> tf(1);
        while (!stop) {
          global.write("imu", imuWithX(1.0));
          global.write("/tf", tf);
        }
      });
  }
  for (int i = 0; i < 30; ++i) {
    global.startRecord("unit");
    global.stopRecord();
  }
  stop = true;
  for (auto & w : writers) {
    w.join();
  }

  EXPECT_FALSE(global.isStarted());
}

TEST_F(RecorderTest, BasicRecorderWorksThroughTheRecorderInterface)
{
  // The driver only sees recorders as recorder::Recorder.
  std::shared_ptr<naoqi::recorder::Recorder> rec = std::make_shared<ProbeRecorder>("imu");
  auto global = std::make_shared<GlobalRecorder>("naoqi_driver");

  EXPECT_FALSE(rec->isInitialized());
  rec->reset(global, 10.f);
  EXPECT_TRUE(rec->isInitialized());
  EXPECT_EQ(rec->topic(), "imu");

  rec->subscribe(true);
  EXPECT_TRUE(rec->isSubscribed());
  rec->setBufferDuration(2.f);
  EXPECT_EQ(std::static_pointer_cast<ProbeRecorder>(rec)->capacity(), 20u);
  EXPECT_NO_THROW(rec->writeDump(node_->now()));
}

TEST(RecorderWindowTest, MessagesOlderThanTheBufferDurationAreOutside)
{
  // A minidump of the last 10 s, taken at t = 100 s.
  const rclcpp::Time dump_time(100, 0, RCL_ROS_TIME);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 95;
  EXPECT_FALSE(naoqi::helpers::recorder::isOutsideWindow(stamp, dump_time, 10.f));
  stamp.sec = 90;
  stamp.nanosec = 500000000;
  EXPECT_FALSE(naoqi::helpers::recorder::isOutsideWindow(stamp, dump_time, 10.f));
  stamp.sec = 80;
  stamp.nanosec = 0;
  EXPECT_TRUE(naoqi::helpers::recorder::isOutsideWindow(stamp, dump_time, 10.f));
}

TEST(RecorderWindowTest, UnstampedMessagesAreKept)
{
  const rclcpp::Time dump_time(100, 0, RCL_ROS_TIME);
  EXPECT_FALSE(
    naoqi::helpers::recorder::isOutsideWindow(builtin_interfaces::msg::Time(), dump_time, 10.f));
}
