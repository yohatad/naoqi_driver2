// ImuConverter against the in-process fake NAOqi: it refuses an IMU whose ALMemory keys
// the robot lacks (Pepper's torso), so that the driver disables it.

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "converters/imu.hpp"
#include "fake_naoqi.hpp"
#include "test_utils.hpp"

using naoqi::converter::ImuConverter;
namespace IMU = naoqi::converter::IMU;

namespace
{

std::vector<std::string> imuKeys(const std::string & device)
{
  std::vector<std::string> keys{"DCM/Time"};
  for (const std::string quantity : {"Angle", "Gyroscope", "Accelerometer"}) {
    for (const std::string axis : {"X", "Y", "Z"}) {
      keys.push_back("Device/SubDeviceList/" + device + "/" + quantity + axis + "/Sensor/Value");
    }
  }
  return keys;
}

}  // namespace

class ImuTest : public naoqi_test::RosTest
{
protected:
  naoqi_test::FakeNaoqi fake_;
};

TEST_F(ImuTest, StartsWhenTheRobotHasAllItsKeys)
{
  fake_.memory->setDataNames(imuKeys("InertialSensorBase"));
  EXPECT_NO_THROW(ImuConverter("imu_base", IMU::BASE, 50.0f, fake_.session));
}

TEST_F(ImuTest, RefusesAnImuWhoseKeysAreMissing)
{
  // What Pepper 1.8 lists for its torso: only the angles X and Y, and DCM/Time.
  fake_.memory->setDataNames(
  {
    "DCM/Time",
    "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value",
    "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value",
  });
  try {
    ImuConverter("imu_torso", IMU::TORSO, 50.0f, fake_.session);
    FAIL() << "expected the torso IMU to be refused";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("7 of its ALMemory keys are missing"), std::string::npos)
      << e.what();
  }
}
