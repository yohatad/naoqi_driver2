// Recorders moved from boost::mutex / boost::shared_ptr to std::mutex / std::shared_ptr.
// These tests pin the buffering behaviour and hammer the locked paths from several
// threads (run them under -fsanitize=thread to turn races into hard failures).

#include <atomic>
#include <filesystem>
#include <memory>
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
  std::size_t size() const { return buffer_.size(); }
  std::size_t capacity() const { return buffer_.capacity(); }
  double frontX() const { return buffer_.front().linear_acceleration.x; }
  double backX() const { return buffer_.back().linear_acceleration.x; }
  double xAt(std::size_t i) const { return buffer_[i].linear_acceleration.x; }
};

Imu imuWithX(double x)
{
  Imu msg;
  msg.linear_acceleration.x = x;
  return msg;
}

} // namespace

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
  for (int i = 0; i < 25; ++i)
    rec.bufferize(imuWithX(i));

  ASSERT_EQ(rec.size(), 10u);
  EXPECT_DOUBLE_EQ(rec.frontX(), 15.0);
  EXPECT_DOUBLE_EQ(rec.backX(), 24.0);
}

TEST_F(RecorderTest, BufferFrequencyDownsamples)
{
  // Converter at 10 Hz, buffer at 5 Hz: every second message is kept.
  // reset() pre-fills the ring (circular_buffer::resize), so it is always full:
  // 10 s * 10 Hz / 2 = 50 slots, the newest kept messages sit at the back.
  ProbeRecorder rec("imu", 5.f);
  rec.reset(std::make_shared<GlobalRecorder>("naoqi_driver"), 10.f);
  ASSERT_EQ(rec.size(), 50u);
  for (int i = 1; i <= 6; ++i)
    rec.bufferize(imuWithX(i));

  EXPECT_DOUBLE_EQ(rec.xAt(rec.size() - 1), 6.0);
  EXPECT_DOUBLE_EQ(rec.xAt(rec.size() - 2), 4.0);
  EXPECT_DOUBLE_EQ(rec.xAt(rec.size() - 3), 2.0);
  EXPECT_DOUBLE_EQ(rec.xAt(rec.size() - 4), 0.0);  // pre-filled default
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
  for (int t = 0; t < 6; ++t)
  {
    producers.emplace_back([&rec, t] {
      for (int i = 0; i < 20000; ++i)
        rec.bufferize(imuWithX(t * 100000 + i));
    });
  }
  std::thread resizer([&] {
    int i = 0;
    while (!stop)
      rec.setBufferDuration(1.f + static_cast<float>(i++ % 5));
  });
  std::thread dumper([&] {
    const auto now = node_->now();
    while (!stop)
      rec.writeDump(now);
  });

  for (auto& p : producers)
    p.join();
  stop = true;
  resizer.join();
  dumper.join();

  EXPECT_LE(rec.size(), rec.capacity());
}

TEST_F(RecorderTest, GlobalRecorderNamesBagInWorkingDirectory)
{
  naoqi_test::TempDir dir(/*chdir_into=*/true);
  GlobalRecorder global("naoqi_driver");

  EXPECT_FALSE(global.isStarted());
  global.startRecord("unit");
  EXPECT_TRUE(global.isStarted());

  const std::string bag = global.stopRecord();
  EXPECT_FALSE(global.isStarted());

  const std::string expected_prefix = std::filesystem::current_path().string() + "/unit_";
  EXPECT_EQ(bag.rfind(expected_prefix, 0), 0u) << bag;
  EXPECT_EQ(bag.substr(bag.size() - 4), ".bag") << bag;
}

TEST_F(RecorderTest, GlobalRecorderStopWithoutStartIsRejected)
{
  GlobalRecorder global("naoqi_driver");
  EXPECT_EQ(global.stopRecord(), "Cannot stop recording while it has not been started.");
}

TEST_F(RecorderTest, GlobalRecorderConcurrentStartStopAndWriteDoNotDeadlock)
{
  naoqi_test::TempDir dir(/*chdir_into=*/true);
  GlobalRecorder global("naoqi_driver");

  std::atomic<bool> stop{false};
  std::vector<std::thread> writers;
  for (int t = 0; t < 4; ++t)
  {
    writers.emplace_back([&] {
      std::vector<geometry_msgs::msg::TransformStamped> tf(1);
      while (!stop)
      {
        global.write("imu", imuWithX(1.0));
        global.write("/tf", tf);
      }
    });
  }
  for (int i = 0; i < 200; ++i)
  {
    global.startRecord("unit");
    global.stopRecord();
  }
  stop = true;
  for (auto& w : writers)
    w.join();

  EXPECT_FALSE(global.isStarted());
}
