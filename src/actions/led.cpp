#include "led.hpp"
#include <rclcpp/rclcpp.hpp>

using RunLed = naoqi_bridge_msgs::action::RunLed;
using RunLedGoalHandle = rclcpp_action::ServerGoalHandle<RunLed>;

namespace naoqi
{
namespace action
{
namespace
{
  struct LedState {
    LedState(rclcpp::Node* node, qi::SessionPtr session) :
      node(node),
      session(std::move(session))
    {}

    rclcpp::Node* node;
    qi::SessionPtr session;
    rclcpp::Logger logger = node->get_logger();
  };

  rclcpp_action::GoalResponse handle_goal(
    std::shared_ptr<LedState> state,
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const RunLed::Goal> goal)
  {
    std::string goal_id = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(state->logger, "Received RunLed goal request %s", goal_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<LedState> state,
    const std::shared_ptr<RunLedGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Received goal cancellation request %s", goal_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    std::shared_ptr<LedState> state,
    const std::shared_ptr<RunLedGoalHandle> goal_handle)
  {
    auto& logger = state->logger;
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(logger, "RunLed action %s starts", goal_id.c_str());
    
    auto result = std::make_shared<RunLed::Result>();
    const auto& session = state->session;
    const auto& goal = goal_handle->get_goal();

    try
    {
      auto leds = session->service("ALLeds").value();
      
      const std::string& target = goal->target;
      const uint8_t mode = goal->mode;
      const float intensity = goal->intensity;
      const auto& color = goal->color;
      const float duration = goal->duration;

      RCLCPP_INFO(logger, "RunLed: target=%s, mode=%d, duration=%.2f", 
                  target.c_str(), mode, duration);

      // Execute based on mode
      switch (mode) {
        case RunLed::Goal::MODE_SET_INTENSITY:
          leds.call<void>("setIntensity", target, intensity);
          break;
          
        case RunLed::Goal::MODE_FADE_INTENSITY:
          leds.call<void>("fade", target, intensity, duration);
          break;
          
        case RunLed::Goal::MODE_RGB:
          leds.call<void>("fadeRGB", target, color.r, color.g, color.b, 0.0f);
          break;
          
        case RunLed::Goal::MODE_RGB_FADE:
          leds.call<void>("fadeRGB", target, color.r, color.g, color.b, duration);
          break;
          
        case RunLed::Goal::MODE_ON:
          leds.call<void>("on", target);
          break;
          
        case RunLed::Goal::MODE_OFF:
          leds.call<void>("off", target);
          break;
          
        default:
          throw std::runtime_error("Invalid LED mode: " + std::to_string(mode));
      }
      
      // Wait for duration if specified
      if (duration > 0.0f && (mode == RunLed::Goal::MODE_FADE_INTENSITY || 
                               mode == RunLed::Goal::MODE_RGB_FADE)) {
        auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(duration));
        rclcpp::sleep_for(sleep_duration);
      }

      // Check if cancelled
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Goal was canceled";
        goal_handle->canceled(result);
        RCLCPP_INFO(logger, "RunLed action %s was canceled", goal_id.c_str());
        return;
      }

      result->success = true;
      result->message = "LED command completed successfully";
      goal_handle->succeed(result);
      RCLCPP_INFO(logger, "RunLed action %s completed", goal_id.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Failed to execute RunLed action %s: %s", 
                   goal_id.c_str(), e.what());
      result->success = false;
      result->message = std::string("Error: ") + e.what();
      goal_handle->abort(result);
    }
  }
}

rclcpp_action::Server<RunLed>::SharedPtr 
createRunLedServer(rclcpp::Node* node, qi::SessionPtr session)
{
  namespace ph = std::placeholders;
  auto state = std::make_shared<LedState>(node, std::move(session));
  return rclcpp_action::create_server<RunLed>(
    node, "run_led",
    std::bind(handle_goal, state, ph::_1, ph::_2),
    std::bind(handle_cancel, state, ph::_1),
    std::bind(handle_accepted, state, ph::_1)
  );
}

} // namespace action
} // namespace naoqi
