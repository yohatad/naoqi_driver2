// Driver::stop() now halts motion and playback through NAOqi. Without a robot we can
// only check that the shutdown path is safe when there is no session; the halt calls
// themselves need the on-robot check.

#include <memory>

#include <gtest/gtest.h>

#include <naoqi_driver/naoqi_driver.hpp>

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
