#include "speech.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <thread>
#include <atomic>

using SpeechWithFeedback = naoqi_bridge_msgs::action::SpeechWithFeedback;
using SpeechGoalHandle = rclcpp_action::ServerGoalHandle<SpeechWithFeedback>;

namespace naoqi
{
namespace action
{
namespace
{
  struct SpeechState {
    SpeechState(rclcpp::Node* node, qi::SessionPtr session) :
      node(node),
      session(std::move(session)),
      logger(node->get_logger())
    {}

    rclcpp::Node* node;
    qi::SessionPtr session;
    rclcpp::Logger logger;
    
    std::mutex mutex;
    std::shared_ptr<SpeechGoalHandle> current_goal;
    qi::AnyObject animated_speech_service;
    std::atomic<bool> cancel_requested{false};
  };

  rclcpp_action::GoalResponse handle_goal(
    std::shared_ptr<SpeechState> state,
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const SpeechWithFeedback::Goal> goal)
  {
    (void)goal;
    std::string goal_id = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(state->logger, "Received SpeechWithFeedback goal request %s", goal_id.c_str());

    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->current_goal) {
      RCLCPP_INFO(
        state->logger,
        "Rejected request %s: robot is already speaking for %s",
        goal_id.c_str(),
        rclcpp_action::to_string(state->current_goal->get_goal_id()).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<SpeechState> state,
    const std::shared_ptr<SpeechGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Received cancel request for %s", goal_id.c_str());
    
    state->cancel_requested = true;
    
    try {
      if (state->animated_speech_service) {
        state->animated_speech_service.call<void>("stopAll");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(state->logger, "Failed to stop speech: %s", e.what());
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    std::shared_ptr<SpeechState> state,
    const std::shared_ptr<SpeechGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Executing SpeechWithFeedback %s", goal_id.c_str());

    // Run speech in a separate thread to avoid blocking
    std::thread([state, goal_handle, goal_id]() {
      try
      {
        // Initialize ALAnimatedSpeech service if needed
        if (!state->animated_speech_service) {
          state->animated_speech_service = state->session->service("ALAnimatedSpeech").value();
        }

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->current_goal = goal_handle;
          state->cancel_requested = false;
        }

        const auto& goal = goal_handle->get_goal();
        std::string text_to_say = goal->say;
        std::string mode = goal->body_language_mode.empty() ? "contextual" : goal->body_language_mode;
        RCLCPP_INFO(state->logger, "Saying: \"%s\" (body_language_mode: %s)", text_to_say.c_str(), mode.c_str());

        // Publish feedback that speech is starting
        if (goal_handle->is_executing() && !goal_handle->is_canceling()) {
          auto feedback = std::make_shared<SpeechWithFeedback::Feedback>();
          goal_handle->publish_feedback(feedback);
        }

        // Build configuration map and call ALAnimatedSpeech
        std::map<std::string, std::string> config{{"bodyLanguageMode", mode}};
        state->animated_speech_service.call<void>("say", text_to_say, config);

        // Check result
        auto result = std::make_shared<SpeechWithFeedback::Result>();
        
        if (state->cancel_requested || goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(state->logger, "Speech cancelled: %s", goal_id.c_str());
        } else if (goal_handle->is_executing()) {
          result->success = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(state->logger, "Speech completed: %s", goal_id.c_str());
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(state->logger, "Speech failed: %s", e.what());
        
        if (goal_handle->is_executing()) {
          auto result = std::make_shared<SpeechWithFeedback::Result>();
          result->success = false;
          goal_handle->abort(result);
        }
      }

      // Cleanup
      std::lock_guard<std::mutex> lock(state->mutex);
      state->current_goal.reset();
      state->cancel_requested = false;
    }).detach();
  }
}

rclcpp_action::Server<SpeechWithFeedback>::SharedPtr 
createSpeechWithFeedbackServer(rclcpp::Node* node, qi::SessionPtr session)
{
  auto state = std::make_shared<SpeechState>(node, std::move(session));
  
  return rclcpp_action::create_server<SpeechWithFeedback>(
    node,
    "naoqi_driver/speech_with_feedback",
    std::bind(handle_goal, state, std::placeholders::_1, std::placeholders::_2),
    std::bind(handle_cancel, state, std::placeholders::_1),
    std::bind(handle_accepted, state, std::placeholders::_1)
  );
}

} // namespace action
} // namespace naoqi