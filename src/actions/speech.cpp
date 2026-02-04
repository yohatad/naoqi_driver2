#include "speech.hpp"
#include <rclcpp/rclcpp.hpp>

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
      session(std::move(session))
    {}

    rclcpp::Node* node;
    qi::SessionPtr session;
    rclcpp::Logger logger = node->get_logger();
    std::shared_ptr<SpeechGoalHandle> current_goal;
    qi::AnyObject tts_service;
    qi::AnyObject memory_service;
    qi::AnyObject text_done_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
  };

  rclcpp_action::GoalResponse handle_goal(
    std::shared_ptr<SpeechState> state,
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const SpeechWithFeedback::Goal> goal)
  {
    std::string goal_id = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(state->logger, "Received SpeechWithFeedback goal request %s", goal_id.c_str());

    if (auto current_goal = state->current_goal) {
      RCLCPP_INFO(
        state->logger,
        "Rejected request %s because the robot is already speaking for request %s",
        goal_id.c_str(),
        rclcpp_action::to_string(current_goal->get_goal_id()).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(state->logger, "Accepted request %s", goal_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void cleanup(std::shared_ptr<SpeechState> state);

  void handle_accepted(
    std::shared_ptr<SpeechState> state,
    const std::shared_ptr<SpeechGoalHandle> goal_handle)
  {
    auto& logger = state->logger;
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(logger, "SpeechWithFeedback action %s starts", goal_id.c_str());
    
    const auto& session = state->session;
    const auto& goal = goal_handle->get_goal();

    try
    {
      // Get TTS service if not already available
      if (!state->tts_service) {
        state->tts_service = session->service("ALTextToSpeech").value();
      }

      // Get memory service for event subscription
      if (!state->memory_service) {
        state->memory_service = session->service("ALMemory").value();
      }

      // Store current goal
      state->current_goal = goal_handle;

      // Subscribe to TextDone event
      state->text_done_subscriber = state->memory_service.call<qi::AnyObject>(
        "subscriber", "ALTextToSpeech/TextDone");
      
      state->text_done_subscriber.connect("signal", [state, goal_handle](const qi::AnyValue& value) {
        bool speech_done = value.to<bool>();
        if (speech_done && goal_handle && goal_handle->is_executing()) {
          auto result = std::make_shared<SpeechWithFeedback::Result>();
          goal_handle->succeed(result);
          RCLCPP_INFO(state->logger, "Speech completed (TextDone event)");
          cleanup(state);
        }
      });

      // Also subscribe to TextStarted for feedback
      auto text_started_subscriber = state->memory_service.call<qi::AnyObject>(
        "subscriber", "ALTextToSpeech/TextStarted");
      
      text_started_subscriber.connect("signal", [state, goal_handle](const qi::AnyValue& value) {
        bool speech_started = value.to<bool>();
        if (speech_started && goal_handle && goal_handle->is_executing()) {
          auto feedback = std::make_shared<SpeechWithFeedback::Feedback>();
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(state->logger, "Speech started (TextStarted event)");
        }
      });

      // Execute the say command
      std::string text_to_say = goal->say;
      RCLCPP_INFO(logger, "Saying: %s", text_to_say.c_str());
      
      // Use async call (fire and forget, we'll get events)
      state->tts_service.async<void>("say", text_to_say);

      // Set up a safety timer in case events don't fire
      state->timer = state->node->create_wall_timer(
        std::chrono::seconds(30),  // Long timeout for long speeches
        [state, goal_handle]() {
          RCLCPP_WARN(state->logger, "Speech action timeout, assuming completion");
          if (goal_handle && goal_handle->is_executing()) {
            auto result = std::make_shared<SpeechWithFeedback::Result>();
            goal_handle->succeed(result);
          }
          cleanup(state);
        }
      );

    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Failed to execute SpeechWithFeedback action %s: %s", 
                   goal_id.c_str(), e.what());
      auto result = std::make_shared<SpeechWithFeedback::Result>();
      goal_handle->abort(result);
      cleanup(state);
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<SpeechState> state,
    const std::shared_ptr<SpeechGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Received goal cancellation request %s", goal_id.c_str());
    
    // Try to stop the speech
    try {
      if (state->tts_service) {
        state->tts_service.call<void>("stopAll");
        RCLCPP_INFO(state->logger, "Stopped speech for goal %s", goal_id.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(state->logger, "Failed to stop speech: %s", e.what());
    }

    // Clean up and cancel
    cleanup(state);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void cleanup(std::shared_ptr<SpeechState> state)
  {
    if (!state->current_goal) {
      return;
    }

    // Cancel timer if active
    if (state->timer) {
      state->timer->cancel();
      state->timer.reset();
    }

    // Disconnect event subscribers
    if (state->text_done_subscriber) {
      state->text_done_subscriber.reset();
    }

    RCLCPP_INFO(state->logger, "Speech action cleaned up");
    state->current_goal.reset();
  }
}

rclcpp_action::Server<SpeechWithFeedback>::SharedPtr 
createSpeechWithFeedbackServer(rclcpp::Node* node, qi::SessionPtr session)
{
  namespace ph = std::placeholders;
  auto state = std::make_shared<SpeechState>(node, std::move(session));
  return rclcpp_action::create_server<SpeechWithFeedback>(
    node, "speech_with_feedback",
    std::bind(handle_goal, state, ph::_1, ph::_2),
    std::bind(handle_cancel, state, ph::_1),
    std::bind(handle_accepted, state, ph::_1)
  );
}

} // namespace action
} // namespace naoqi
