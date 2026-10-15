// The listen action against the in-process fake NAOqi, driven by a real action client.

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <naoqi_bridge_msgs/action/listen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "actions/listen.hpp"
#include "actions/listen_validation.hpp"
#include "fake_naoqi.hpp"
#include "helpers/parameter_helpers.hpp"
#include "test_utils.hpp"

using Listen = naoqi_bridge_msgs::action::Listen;
using GoalHandle = rclcpp_action::ClientGoalHandle<Listen>;
using namespace std::chrono_literals;

class ListenTest : public naoqi_test::RosTest
{
protected:
  void startServer(double timeout_s)
  {
    naoqi::helpers::declareNumberParameter(node_.get(), "listen_timeout", 30.0, "");
    node_->set_parameter(rclcpp::Parameter("listen_timeout", timeout_s));
    server_ = naoqi::action::createListenServer(node_.get(), fake_.session);
    client_ = rclcpp_action::create_client<Listen>(node_, "naoqi_driver/listen");
    executor_.add_node(node_);
    spinner_ = std::thread([this] {executor_.spin();});
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.remove_node(node_);
    // After the executor: as in the driver, the server must not be destroyed while
    // the executor (rclcpp, uninstrumented) may hold the last reference to it.
    server_.reset();  // aborts a goal still waiting, joins its thread
  }

  GoalHandle::SharedPtr send(
    const std::vector<std::string> & expected,
    const std::string & language = "")
  {
    Listen::Goal goal;
    goal.expected = expected;
    goal.language = language;
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  GoalHandle::WrappedResult result(const GoalHandle::SharedPtr & goal, std::chrono::seconds timeout)
  {
    auto future = client_->async_get_result(goal);
    EXPECT_EQ(future.wait_for(timeout), std::future_status::ready);
    return future.get();
  }

  naoqi_test::FakeNaoqi fake_;
  std::unique_ptr<naoqi::action::ActionHandle> server_;
  rclcpp_action::Client<Listen>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
};

TEST_F(ListenTest, AnswerCompletesTheGoalAndRemovesTheTopic)
{
  startServer(30.0);
  auto goal = send({"yes", "no"});
  ASSERT_TRUE(goal);
  ASSERT_TRUE(fake_.log.waitFor("subscriber(ros_")) << "dialog setup did not finish";
  EXPECT_EQ(fake_.log.count("loadTopicContent("), 1u);

  // The driver connects to the memory signal just after subscriber() returns, so keep
  // "speaking" until it listens, as a person would.
  auto result_future = client_->async_get_result(goal);
  for (int i = 0; i < 50 && result_future.wait_for(100ms) != std::future_status::ready; ++i) {
    fake_.memory->emit("ros_", "yes");
  }
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto r = result_future.get();
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_EQ(r.result->result.size(), 1u);
  EXPECT_EQ(r.result->result[0], "yes");
  EXPECT_TRUE(fake_.log.waitFor("unloadTopic(ros_"));
}

TEST_F(ListenTest, SilenceAbortsTheGoalAfterTheTimeout)
{
  startServer(0.5);
  auto goal = send({"yes"});
  ASSERT_TRUE(goal);
  const auto r = result(goal, 5s);
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_TRUE(fake_.log.waitFor("deactivateTopic(ros_"));
}

TEST_F(ListenTest, CancelEndsTheGoal)
{
  startServer(30.0);
  auto goal = send({"yes"});
  ASSERT_TRUE(goal);
  ASSERT_TRUE(fake_.log.waitFor("subscriber(ros_"));
  client_->async_cancel_goal(goal);
  const auto r = result(goal, 5s);
  EXPECT_EQ(r.code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(ListenTest, PhrasesQiChatCannotHoldAreRejected)
{
  startServer(30.0);
  EXPECT_FALSE(send({"say \"hi\""}));
  EXPECT_FALSE(send({"two\nlines"}));
  EXPECT_FALSE(send({""}));
  EXPECT_FALSE(send({"yes"}, "xx"));
  EXPECT_EQ(fake_.log.count("loadTopicContent("), 0u);
}

TEST(ListenValidation, LanguageCodes)
{
  EXPECT_EQ(naoqi::action::qichatLanguage("en"), "English");
  EXPECT_EQ(naoqi::action::qichatLanguage("ja"), "Japanese");
  EXPECT_EQ(naoqi::action::qichatLanguage("xx"), "");
}
