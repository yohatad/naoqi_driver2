#include "listen.hpp"
#include "listen_validation.hpp"
#include "helpers/parameter_helpers.hpp"
#include "helpers/string_helpers.hpp"
#include <chrono>
#include <mutex>
#include <thread>

using Listen = naoqi_bridge_msgs::action::Listen;
using ListenGoalHandle = rclcpp_action::ServerGoalHandle<Listen>;

namespace naoqi
{
namespace action
{
namespace
{

struct ListenState
{
  ListenState(rclcpp::Node * node, qi::SessionPtr session)
  : node(node),
    session(std::move(session))
  {}

  rclcpp::Node * node;
  qi::SessionPtr session;
  rclcpp::Logger logger = node->get_logger();
  std::mutex mutex;
  // Guarded by mutex. Only the thread that claims the goal via claimGoal
  // may drive it to a terminal state.
  std::shared_ptr<ListenGoalHandle> current_goal;
  qi::AnyObject memory_subscriber;
  double timeout_s = 0.0;  // listen_timeout; 0 waits forever
  GoalWorker worker;  // joined at shutdown; see goal_worker.hpp
};

std::string topicNameForGoalId(const rclcpp_action::GoalUUID & goal_id)
{
  std::string topic_name = rclcpp_action::to_string(goal_id);
  naoqi::helpers::string::replaceAll(topic_name, "-", "");
  return std::string("ros_") + topic_name;
}

struct ClaimedGoal
{
  std::shared_ptr<ListenGoalHandle> goal;
  qi::AnyObject memory_subscriber;
};

// The dialog callback (NAOqi thread) and handleCancel (ROS executor) can
// both try to finish the same goal. Whoever claims it first owns the
// terminal transition; the other backs off.
/** Takes over whichever goal is running, for example to abort it at shutdown. */
ClaimedGoal claimCurrentGoal(ListenState & state)
{
  std::lock_guard<std::mutex> lock(state.mutex);
  ClaimedGoal claimed;
  claimed.goal = std::move(state.current_goal);
  claimed.memory_subscriber = state.memory_subscriber;
  state.current_goal.reset();
  state.memory_subscriber.reset();
  return claimed;
}

ClaimedGoal claimGoal(ListenState & state, const std::shared_ptr<ListenGoalHandle> & expected)
{
  std::lock_guard<std::mutex> lock(state.mutex);
  if (!state.current_goal || state.current_goal != expected) {
    return {};
  }
  ClaimedGoal claimed;
  claimed.goal = std::move(state.current_goal);
  claimed.memory_subscriber = state.memory_subscriber;
  state.current_goal.reset();
  state.memory_subscriber.reset();
  return claimed;
}

rclcpp_action::GoalResponse handleGoal(
  std::shared_ptr<ListenState> state,
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const Listen::Goal> goal)
{
  std::string goal_id = rclcpp_action::to_string(uuid);
  RCLCPP_INFO(state->logger, "Received goal request %s", goal_id.c_str());

  const std::string error = validateListenGoal(*goal);
  if (!error.empty()) {
    RCLCPP_WARN(state->logger, "Rejected request %s: %s", goal_id.c_str(), error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::shared_ptr<ListenGoalHandle> current_goal;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    current_goal = state->current_goal;
  }
  if (current_goal) {
    RCLCPP_INFO(
      state->logger,
      "Rejected request %s because the robot is already listening for request %s",
      goal_id.c_str(),
      rclcpp_action::to_string(current_goal->get_goal_id()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(state->logger, "Accepted request %s", goal_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void cleanupDialog(ListenState & state, const std::string & topic_name);

void waitForAnswer(
  const std::shared_ptr<ListenState> & state,
  const std::shared_ptr<ListenGoalHandle> & goal_handle,
  const std::string & topic_name);

void runListen(
  std::shared_ptr<ListenState> state,
  const std::shared_ptr<ListenGoalHandle> goal_handle)
{
  auto & logger = state->logger;
  std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(logger, "Listen %s starts", goal_id.c_str());
  auto result = std::make_shared<Listen::Result>();

  const auto & session = state->session;
  const auto topic_name = topicNameForGoalId(goal_handle->get_goal_id());

  try {
    auto dialog = session->service("ALDialog").value();

    try {
      session->service("ALSpeechRecognition").value().call<void>("_enableFreeSpeechToText");
    } catch (const std::exception & e) {
      RCLCPP_INFO(logger, "Failed to enable free speech to text: %s", e.what());
    }

    // Get language from goal or from dialog
    const auto & iso_language = goal_handle->get_goal()->language;
    const std::string language = [&]
      {
        if (iso_language.empty()) {
          return dialog.call<std::string>("getLanguage");
        } else {
          return qichatLanguage(iso_language);
        }
      }();

    // Setup topic
    std::stringstream pattern_ss;
    pattern_ss << "[ ";
    for (const auto & utterance: goal_handle->get_goal()->expected) {
      pattern_ss << "\"" << utterance << "\" ";
    }
    pattern_ss << "]";

    std::stringstream topic_ss;
    topic_ss << "topic: ~" << topic_name << " ()" << std::endl
             << "language: " << language << std::endl
             << "u:(_" << pattern_ss.str() << ") $" << topic_name << "/result=$1" << std::endl
             << "u:(_*) $" << topic_name << "/result=$1" << std::endl;
    std::string qichat_topic = topic_ss.str();
    RCLCPP_INFO(logger, "Loading topic:\n%s", qichat_topic.c_str());
    dialog.call<void>("loadTopicContent", qichat_topic);
    dialog.call<void>("activateTopic", topic_name);
    dialog.call<void>("subscribe", topic_name);
    dialog.call<void>("setFocus", topic_name);

    auto memory = session->service("ALMemory").value();
    auto memory_subscriber = memory.call<qi::AnyObject>("subscriber", topic_name + "/result");
    memory_subscriber.connect(
      "signal", [ = ](const qi::AnyValue & value)
      {
        // This runs on a NAOqi callback thread: nothing may escape, or the
        // whole process terminates.
        try {
          auto utterance = value.toString();
          RCLCPP_INFO(state->logger, "Received input: %s", utterance.c_str());

          auto claimed = claimGoal(*state, goal_handle);
          if (!claimed.goal) {
            // Already cancelled (or not yet registered); drop the utterance.
            return;
          }
          claimed.memory_subscriber.reset();
          cleanupDialog(*state, topic_name);

          result->result = std::vector<std::string>{utterance};
          goal_handle->succeed(result);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(state->logger, "Error while finishing listen goal: %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(state->logger, "Unknown error while finishing listen goal");
        }
      });

    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->current_goal == goal_handle) {  // not answered or cancelled meanwhile
        state->memory_subscriber = memory_subscriber;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to start listen %s: %s", goal_id.c_str(), e.what());
    auto claimed = claimGoal(*state, goal_handle);
    if (claimed.goal) {
      claimed.memory_subscriber.reset();
      cleanupDialog(*state, topic_name);
      goal_handle->abort(result);
    }
    return;
  }

  waitForAnswer(state, goal_handle, topic_name);
}

/** Returns once the goal was answered or cancelled, or aborts it after listen_timeout. */
void waitForAnswer(
  const std::shared_ptr<ListenState> & state,
  const std::shared_ptr<ListenGoalHandle> & goal_handle,
  const std::string & topic_name)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(state->timeout_s));
  while (!state->worker.stopping()) {  // at shutdown the handle aborts the goal
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->current_goal != goal_handle) {
        return;  // answered or cancelled
      }
    }
    if (state->timeout_s > 0.0 && std::chrono::steady_clock::now() >= deadline) {
      auto claimed = claimGoal(*state, goal_handle);
      if (claimed.goal) {
        claimed.memory_subscriber.reset();
        cleanupDialog(*state, topic_name);
        RCLCPP_WARN(
          state->logger, "Heard nothing within listen_timeout (%.1f s), aborting the goal",
          state->timeout_s);
        goal_handle->abort(std::make_shared<Listen::Result>());
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void handleAccepted(
  std::shared_ptr<ListenState> state,
  const std::shared_ptr<ListenGoalHandle> goal_handle)
{
  // Register the goal right away: a concurrent goal is rejected, and an answer that
  // arrives while the dialog is being set up is not lost.
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->current_goal = goal_handle;
  }
  // Setting up ALDialog takes several calls; keep them off the ROS callback threads.
  state->worker.start([state, goal_handle] {runListen(state, goal_handle);});
}

rclcpp_action::CancelResponse handleCancel(
  std::shared_ptr<ListenState> state,
  const std::shared_ptr<ListenGoalHandle> goal_handle)
{
  std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(state->logger, "Received goal cancellation request %s", goal_id.c_str());

  auto claimed = claimGoal(*state, goal_handle);
  if (!claimed.goal) {
    // The dialog callback already finished (or is finishing) this goal.
    return rclcpp_action::CancelResponse::REJECT;
  }

  claimed.memory_subscriber.reset();
  cleanupDialog(*state, topicNameForGoalId(goal_handle->get_goal_id()));

  // The goal only enters CANCELING once we return ACCEPT, so the terminal
  // transition has to happen on another thread, after that.
  state->worker.start(
    [state, goal_handle]
    {
      try {
        for (int i = 0; i < 100 && !goal_handle->is_canceling() && !state->worker.stopping(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        goal_handle->canceled(std::make_shared<Listen::Result>());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          state->logger, "Failed to mark listen %s as canceled: %s",
          rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), e.what());
      }
    });

  return rclcpp_action::CancelResponse::ACCEPT;
}

void cleanupDialog(ListenState & state, const std::string & topic_name)
{
  auto & logger = state.logger;
  const auto & session = state.session;
  try {
    auto dialog = session->service("ALDialog").value();

    // ALDialog.unsubscribe(topic_name) is deliberately not called.

    try {
      dialog.call<void>("deactivateTopic", topic_name);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Failed to deactivate topic %s: %s", topic_name.c_str(), e.what());
    }

    try {
      dialog.call<void>("unloadTopic", topic_name);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "Failed to unload topic %s: %s", topic_name.c_str(), e.what());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to get ALDialog service while cleaning up: %s", e.what());
  }

  RCLCPP_INFO(logger, "Listen topic %s stopped and cleaned up", topic_name.c_str());
}
}  // namespace

std::unique_ptr<ActionHandle>
createListenServer(rclcpp::Node * node, qi::SessionPtr session)
{
  namespace ph = std::placeholders;
  auto task = std::make_shared<ListenState>(node, std::move(session));
  task->timeout_s = helpers::declareNumberParameter(
    node, "listen_timeout", 30.0,
    "Seconds to wait for speech before a listen goal is aborted; 0 waits forever");
  auto server = rclcpp_action::create_server<Listen>(
    node, "naoqi_driver/listen",
    std::bind(handleGoal, task, ph::_1, ph::_2),
    std::bind(handleCancel, task, ph::_1),
    std::bind(handleAccepted, task, ph::_1)
  );
  return std::make_unique<ActionHandle>(
    server, [task]() {
      task->worker.requestStop();
      // Abort a goal still waiting for speech and remove its dialog topic.
      auto claimed = claimCurrentGoal(*task);
      if (claimed.goal) {
        claimed.memory_subscriber.reset();
        cleanupDialog(*task, topicNameForGoalId(claimed.goal->get_goal_id()));
        try {
          claimed.goal->abort(std::make_shared<Listen::Result>());
        } catch (const std::exception & e) {
          RCLCPP_WARN(task->logger, "Failed to abort listen goal on shutdown: %s", e.what());
        }
      }
      task->worker.join();
    });
}

}  // namespace action
}  // namespace naoqi
