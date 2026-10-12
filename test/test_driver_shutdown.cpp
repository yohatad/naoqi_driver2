// Driver::stop() now halts motion and playback through NAOqi. Without a robot we can
// only check that the shutdown path is safe when there is no session; the halt calls
// themselves need the on-robot check.

#include <memory>

#include <gtest/gtest.h>

#include <naoqi_driver/naoqi_driver.hpp>

#include "fake_naoqi.hpp"
#include "test_utils.hpp"

class DriverShutdownTest : public naoqi_test::RosTest {};

TEST_F(DriverShutdownTest, StopWithoutSessionDoesNotThrow)
{
  auto driver = std::make_shared<naoqi::Driver>();
  EXPECT_NO_THROW(driver->stop());
}

TEST_F(DriverShutdownTest, StopIsIdempotent)
{
  auto driver = std::make_shared<naoqi::Driver>();
  driver->requestStop();
  EXPECT_NO_THROW(driver->stop());
  EXPECT_NO_THROW(driver->stop());
}

TEST_F(DriverShutdownTest, StopHaltsMotionSpeechAndAudioThroughNaoqi)
{
  // Ctrl+C runs Driver::stop(); ALMotion.move() would otherwise keep the last velocity.
  naoqi_test::FakeNaoqi fake;
  auto driver = std::make_shared<naoqi::Driver>();
  driver->setQiSession(fake.session);
  naoqi_test::LogCapture logs;
  driver->stop();

  EXPECT_EQ(fake.log.count("stopMove()"), 1u);
  EXPECT_EQ(fake.log.count("ALTextToSpeech.stopAll()"), 1u);  // also ends animated speech
  EXPECT_EQ(fake.log.count("ALAudioPlayer.stopAll()"), 1u);
  // Every call must exist on NAOqi 2.5, which has no ALAnimatedSpeech.stopAll.
  EXPECT_FALSE(logs.contains("failed on shutdown"));
}
