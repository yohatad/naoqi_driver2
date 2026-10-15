// The SpeechWithFeedback action against the fake NAOqi, whose ALAnimatedSpeech has no
// stopAll, as on NAOqi 2.5: a cancel or a shutdown must still stop the speech.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <naoqi_bridge_msgs/action/speech_with_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "actions/speech.hpp"
#include "fake_naoqi.hpp"
#include "test_utils.hpp"

using Speech = naoqi_bridge_msgs::action::SpeechWithFeedback;
using GoalHandle = rclcpp_action::ClientGoalHandle<Speech>;
using namespace std::chrono_literals;

class SpeechTest : public naoqi_test::RosTest
{
protected:
  void SetUp() override
  {
    server_ = naoqi::action::createSpeechWithFeedbackServer(node_.get(), fake_.session);
    client_ = rclcpp_action::create_client<Speech>(node_, "naoqi_driver/speech_with_feedback");
    executor_.add_node(node_);
    spinner_ = std::thread([this] {executor_.spin();});
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    stopSpinning();
    server_.reset();
  }

  /** Stops the executor first, as the driver does before destroying its servers. */
  void stopSpinning()
  {
    if (!spinner_.joinable()) {
      return;
    }
    executor_.cancel();
    spinner_.join();
    executor_.remove_node(node_);
  }

  GoalHandle::SharedPtr say(const std::string & text)
  {
    Speech::Goal goal;
    goal.say = text;
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  naoqi_test::FakeNaoqi fake_;
  std::unique_ptr<naoqi::action::ActionHandle> server_;
  rclcpp_action::Client<Speech>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

TEST_F(SpeechTest, SucceedsWhenTheSentenceEnds)
{
  auto goal = say("hello");
  ASSERT_TRUE(goal);
  auto result = client_->async_get_result(goal);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(fake_.log.count("ALAnimatedSpeech.say(hello)"), 1u);
}

TEST_F(SpeechTest, CancelStopsTheSpeech)
{
  fake_.voice->duration_ms = 10000;  // a long sentence
  auto goal = say("a long sentence");
  ASSERT_TRUE(goal);
  ASSERT_TRUE(fake_.log.waitFor("ALAnimatedSpeech.say("));
  auto result = client_->async_get_result(goal);
  const auto start = std::chrono::steady_clock::now();
  client_->async_cancel_goal(goal);
  ASSERT_EQ(result.wait_for(5s), std::future_status::ready);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 3s) << "the speech did not stop";
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(fake_.log.count("ALTextToSpeech.stopAll()"), 1u);
}

TEST_F(SpeechTest, ShutdownStopsTheSpeech)
{
  fake_.voice->duration_ms = 10000;
  auto goal = say("a long sentence");
  ASSERT_TRUE(goal);
  ASSERT_TRUE(fake_.log.waitFor("ALAnimatedSpeech.say("));
  stopSpinning();
  const auto start = std::chrono::steady_clock::now();
  server_.reset();  // what Ctrl+C on the driver does, after stopping the executor
  EXPECT_LT(std::chrono::steady_clock::now() - start, 3s) << "the speech did not stop";
  EXPECT_EQ(fake_.log.count("ALTextToSpeech.stopAll()"), 1u);
}
