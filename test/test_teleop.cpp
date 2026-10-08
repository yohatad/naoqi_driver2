// TeleopSubscriber against the in-process fake NAOqi: what reaches ALMotion for each
// command, including the /cmd_vel watchdog and the joint-command validation.

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <naoqi_bridge_msgs/msg/joint_angles_trajectory.hpp>
#include <naoqi_bridge_msgs/msg/joint_angles_with_speed.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake_naoqi.hpp"
#include "subscribers/teleop.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;

class TeleopTest : public naoqi_test::RosTest
{
protected:
  void SetUp() override
  {
    teleop_ = std::make_shared<naoqi::subscriber::TeleopSubscriber>(
      "teleop", "/test_cmd_vel", "/test_joint_angles", "/test_joint_trajectory", fake_.session);
    teleop_->reset(node_.get());
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/test_cmd_vel", 10);
    angles_pub_ = node_->create_publisher<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
      "/test_joint_angles", 10);
    traj_pub_ = node_->create_publisher<naoqi_bridge_msgs::msg::JointAnglesTrajectory>(
      "/test_joint_trajectory", 10);
    executor_.add_node(node_);
    spin_for(200ms);  // let the subscriptions match
  }

  void TearDown() override {executor_.remove_node(node_);}

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some(10ms);
      std::this_thread::sleep_for(5ms);
    }
  }

  /** Spins until ALMotion received a call starting with prefix. */
  bool spin_until_call(const std::string & prefix, std::chrono::milliseconds timeout = 2s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (fake_.log.count(prefix) > 0) {
        return true;
      }
      executor_.spin_some(10ms);
    }
    return fake_.log.count(prefix) > 0;
  }

  static geometry_msgs::msg::Twist twist(double x, double y, double theta)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = x;
    t.linear.y = y;
    t.angular.z = theta;
    return t;
  }

  naoqi_test::FakeNaoqi fake_;
  std::shared_ptr<naoqi::subscriber::TeleopSubscriber> teleop_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>::SharedPtr angles_pub_;
  rclcpp::Publisher<naoqi_bridge_msgs::msg::JointAnglesTrajectory>::SharedPtr traj_pub_;
};

TEST_F(TeleopTest, VelocityIsForwardedToMove)
{
  twist_pub_->publish(twist(0.1, 0.0, 0.2));
  ASSERT_TRUE(spin_until_call("move(0.1,0,0.2)"));
}

TEST_F(TeleopTest, NonFiniteVelocityNeverReachesAlMotion)
{
  twist_pub_->publish(twist(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("move("), 0u);
}

TEST_F(TeleopTest, SilentCmdVelStopsTheBase)
{
  twist_pub_->publish(twist(0.1, 0.0, 0.0));
  ASSERT_TRUE(spin_until_call("move(0.1,0,0)"));
  // No more commands: after cmd_vel_timeout (0.5 s) the watchdog sends a zero move.
  EXPECT_TRUE(spin_until_call("move(0,0,0)", 2s));
  const size_t stops = fake_.log.count("move(0,0,0)");
  spin_for(700ms);
  EXPECT_EQ(fake_.log.count("move(0,0,0)"), stops) << "the watchdog must stop the base once";
}

TEST_F(TeleopTest, ValidJointAnglesAreForwarded)
{
  naoqi_bridge_msgs::msg::JointAnglesWithSpeed msg;
  msg.joint_names = {"HeadYaw"};
  msg.joint_angles = {0.25f};
  msg.speed = 0.1f;
  angles_pub_->publish(msg);
  EXPECT_TRUE(spin_until_call("setAngles(HeadYaw;0.25;0.1)"));
}

TEST_F(TeleopTest, InvalidJointAnglesAreRejected)
{
  naoqi_bridge_msgs::msg::JointAnglesWithSpeed msg;
  msg.joint_names = {"HeadYaw"};
  msg.joint_angles = {0.25f};
  msg.speed = 1.5f;
  angles_pub_->publish(msg);
  msg.speed = 0.1f;
  msg.joint_angles = {std::numeric_limits<float>::quiet_NaN()};
  angles_pub_->publish(msg);
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("setAngles("), 0u);
}

TEST_F(TeleopTest, RelativeBezierTrajectoryIsRejectedAbsoluteRuns)
{
  naoqi_bridge_msgs::msg::JointAnglesTrajectory traj;
  traj.joint_names = {"HeadYaw"};
  traj.joint_angles = {0.3f};
  traj.times = {1.0f};
  traj.relative = 1;
  traj.use_bezier = true;
  traj_pub_->publish(traj);
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("angleInterpolationBezier("), 0u);

  traj.relative = 0;
  traj_pub_->publish(traj);
  EXPECT_TRUE(spin_until_call("angleInterpolationBezier("));
}

TEST_F(TeleopTest, SpeedCapScalesCommandsDown)
{
  node_->set_parameter(rclcpp::Parameter("max_linear_speed", 0.05));
  teleop_->reset(node_.get());
  spin_for(200ms);
  twist_pub_->publish(twist(0.1, 0.0, 0.0));
  EXPECT_TRUE(spin_until_call("move(0.05,0,0)"));
  node_->set_parameter(rclcpp::Parameter("max_linear_speed", 0.0));
}

TEST_F(TeleopTest, NaoqiErrorsAreLoggedNotLost)
{
  naoqi_test::LogCapture logs;
  naoqi_bridge_msgs::msg::JointAnglesWithSpeed msg;
  msg.joint_names = {"BadJoint"};
  msg.joint_angles = {0.1f};
  msg.speed = 0.1f;
  angles_pub_->publish(msg);
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (!logs.contains("ALMotion.setAngles failed") && std::chrono::steady_clock::now() < end) {
    executor_.spin_some(10ms);
  }
  EXPECT_TRUE(logs.contains("ALMotion.setAngles failed: Unknown joint: BadJoint"));
}
