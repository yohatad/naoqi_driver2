// MovetoSubscriber against the in-process fake NAOqi: frames, distance cap, bad input.

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "fake_naoqi.hpp"
#include "subscribers/moveto.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;

class MovetoTest : public naoqi_test::RosTest
{
protected:
  void SetUp() override
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    moveto_ = std::make_shared<naoqi::subscriber::MovetoSubscriber>(
      "moveto", "/test_moveto", fake_.session, buffer_);
    moveto_->reset(node_.get());
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/test_moveto", 10);
    executor_.add_node(node_);
    spin_for(200ms);
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

  void send(const std::string & frame, double x, double y)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = frame;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    pub_->publish(goal);
  }

  naoqi_test::FakeNaoqi fake_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<naoqi::subscriber::MovetoSubscriber> moveto_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

TEST_F(MovetoTest, GoalInBaseFootprintIsSent)
{
  send("base_footprint", 0.5, 0.0);
  spin_for(300ms);
  EXPECT_TRUE(fake_.log.waitFor("moveTo(0.5,0,0)"));
}

TEST_F(MovetoTest, GoalBeyondTheCapIsRejected)
{
  naoqi_test::LogCapture logs;
  send("base_footprint", 100.0, 0.0);  // default cap is 3 m
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("moveTo("), 0u);
  EXPECT_TRUE(logs.contains("moveto_max_distance"));
}

TEST_F(MovetoTest, NonFiniteGoalIsRejected)
{
  send("base_footprint", std::numeric_limits<double>::quiet_NaN(), 0.0);
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("moveTo("), 0u);
}

TEST_F(MovetoTest, OdomGoalIsTransformedIntoBaseFootprint)
{
  geometry_msgs::msg::TransformStamped odom_in_base;  // robot standing at x = 1 in odom
  odom_in_base.header.frame_id = "base_footprint";
  odom_in_base.child_frame_id = "odom";
  odom_in_base.transform.translation.x = -1.0;
  odom_in_base.transform.rotation.w = 1.0;
  buffer_->setTransform(odom_in_base, "test", true);

  send("odom", 1.5, 0.0);  // 0.5 m ahead of the robot
  spin_for(300ms);
  EXPECT_TRUE(fake_.log.waitFor("moveTo(0.5,0,0)"));
}

TEST_F(MovetoTest, OdomGoalWithoutTransformIsDroppedWithAWarning)
{
  naoqi_test::LogCapture logs;
  send("odom", 0.5, 0.0);
  spin_for(300ms);
  EXPECT_EQ(fake_.log.count("moveTo("), 0u);
  EXPECT_TRUE(logs.contains("Cannot transform the moveto goal"));
}
