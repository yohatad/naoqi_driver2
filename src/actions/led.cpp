#include "led.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <thread>
#include <chrono>

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
      session(std::move(session)),
      logger(node->get_logger())
    {}

    rclcpp::Node* node;
    qi::SessionPtr session;
    rclcpp::Logger logger;
    std::mutex mutex;
    std::shared_ptr<RunLedGoalHandle> current_goal;
    std::string current_target;
  };

  rclcpp_action::GoalResponse handle_goal(
    std::shared_ptr<LedState> state,
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const RunLed::Goal> goal)
  {
    (void)goal;
    std::string goal_id = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(state->logger, "Received RunLed goal request %s", goal_id.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<LedState> state,
    const std::shared_ptr<RunLedGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Received cancel request for RunLed %s", goal_id.c_str());

    std::lock_guard<std::mutex> lock(state->mutex);
    try {
      if (!state->current_target.empty()) {
        auto leds = state->session->service("ALLeds").value();
        leds.call<void>("off", state->current_target);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(state->logger, "Failed to stop LED on cancel: %s", e.what());
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    std::shared_ptr<LedState> state,
    const std::shared_ptr<RunLedGoalHandle> goal_handle)
  {
    std::thread([state, goal_handle]() {
      std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
      auto result = std::make_shared<RunLed::Result>();

      try
      {
        auto leds = state->session->service("ALLeds").value();
        const auto& goal = goal_handle->get_goal();
        const std::string& target = goal->target;
        const float intensity = goal->intensity;
        const auto& color = goal->color;
        const float duration = goal->duration;

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->current_goal = goal_handle;
          state->current_target = target;
        }

        RCLCPP_INFO(state->logger, "RunLed: target=%s, mode=%d, duration=%.2f",
                    target.c_str(), goal->mode, duration);

        switch (goal->mode) {
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
            throw std::runtime_error("Invalid LED mode: " + std::to_string(goal->mode));
        }

        // For fade modes, sleep and publish elapsed_time feedback while waiting.
        // handle_cancel will call off() on the target to interrupt the fade early.
        if (duration > 0.0f &&
            (goal->mode == RunLed::Goal::MODE_FADE_INTENSITY ||
             goal->mode == RunLed::Goal::MODE_RGB_FADE))
        {
          const auto start = std::chrono::steady_clock::now();
          const auto end   = start + std::chrono::duration<float>(duration);

          while (std::chrono::steady_clock::now() < end) {
            if (goal_handle->is_canceling()) break;

            auto feedback = std::make_shared<RunLed::Feedback>();
            feedback->elapsed_time = std::chrono::duration<float>(
              std::chrono::steady_clock::now() - start).count();
            goal_handle->publish_feedback(feedback);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }

        std::lock_guard<std::mutex> lock(state->mutex);
        state->current_goal.reset();
        state->current_target.clear();

        if (goal_handle->is_canceling()) {
          result->success = false;
          result->message = "Cancelled";
          goal_handle->canceled(result);
          RCLCPP_INFO(state->logger, "RunLed %s cancelled", goal_id.c_str());
        } else {
          result->success = true;
          result->message = "OK";
          goal_handle->succeed(result);
          RCLCPP_INFO(state->logger, "RunLed %s completed", goal_id.c_str());
        }
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(state->logger, "RunLed %s failed: %s", goal_id.c_str(), e.what());
        std::lock_guard<std::mutex> lock(state->mutex);
        state->current_goal.reset();
        state->current_target.clear();
        result->success = false;
        result->message = e.what();
        goal_handle->abort(result);
      }
    }).detach();
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
