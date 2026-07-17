#include "listen.hpp"
#include <boost/algorithm/string.hpp>
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
  // Trivial Qi Object with a callback to receive ALMemory events
  struct MemoryReceiver
  {
    void callback(const std::string& key, const qi::AnyValue& value)
    {
      if (key == "Dialog/Answered")
      {
        std::cout << "Received input: " << value.toString() << std::endl;
      }
    }
  };

  QI_REGISTER_OBJECT(MemoryReceiver, callback);

  std::string qichatLanguageFromIso(const std::string& iso_language)
  {
    static const std::unordered_map<std::string, std::string> iso_to_qichat = {
      {"en", "English"},
      {"fr", "French"},
      {"de", "German"},
      {"it", "Italian"},
      {"ja", "Japanese"},
      {"ko", "Korean"},
      {"pt", "Portuguese"},
      {"es", "Spanish"},
      {"zh", "Chinese"}
    };

    auto it = iso_to_qichat.find(iso_language);
    if (it == iso_to_qichat.end()) {
      throw std::runtime_error("Unsupported language code: " + iso_language);
    }
    return it->second;
  }

  struct ListenState {
    ListenState(rclcpp::Node* node, qi::SessionPtr session) :
      node(node),
      session(std::move(session))
    {}

    rclcpp::Node* node;
    qi::SessionPtr session;
    rclcpp::Logger logger = node->get_logger();
    std::mutex mutex;
    // Guarded by mutex. Only the thread that claims the goal via claim_goal
    // may drive it to a terminal state.
    std::shared_ptr<ListenGoalHandle> current_goal;
    qi::AnyObject memory_subscriber;
  };

  std::string topic_name_for_goal_id(const rclcpp_action::GoalUUID& goal_id)
  {
    std::string topic_name = rclcpp_action::to_string(goal_id);
    boost::algorithm::replace_all(topic_name, "-", "");
    return std::string("ros_") + topic_name;
  }

  struct ClaimedGoal {
    std::shared_ptr<ListenGoalHandle> goal;
    qi::AnyObject memory_subscriber;
  };

  // The dialog callback (NAOqi thread) and handle_cancel (ROS executor) can
  // both try to finish the same goal. Whoever claims it first owns the
  // terminal transition; the other backs off.
  ClaimedGoal claim_goal(ListenState& state, const std::shared_ptr<ListenGoalHandle>& expected)
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

  rclcpp_action::GoalResponse handle_goal(
    std::shared_ptr<ListenState> state,
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const Listen::Goal> goal)
  {
    std::string goal_id = rclcpp_action::to_string(uuid);
    RCLCPP_INFO(state->logger, "Received goal request %s", goal_id.c_str());

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

  void cleanup_dialog(ListenState& state, const std::string& topic_name);

  void handle_accepted(
    std::shared_ptr<ListenState> state,
    const std::shared_ptr<ListenGoalHandle> goal_handle)
  {
    auto& logger = state->logger;
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(logger, "Listen %s starts", goal_id.c_str());
    auto result = std::make_shared<Listen::Result>();

    const auto& session = state->session;
    boost::algorithm::replace_all(goal_id, "-", "");
    auto topic_name = std::string("ros_") + goal_id;

    try
    {
      auto dialog = session->service("ALDialog").value();

      try
      {
        session->service("ALSpeechRecognition").value().call<void>("_enableFreeSpeechToText");
      }
      catch (const std::exception& e)
      {
        RCLCPP_INFO(logger, "Failed to enable free speech to text: %s", e.what());
      }

      // Get language from goal or from dialog
      const auto& iso_language = goal_handle->get_goal()->language;
      const std::string language = [&]
      {
        if (iso_language.empty())
        {
          return dialog.call<std::string>("getLanguage");
        }
        else
        {
          return qichatLanguageFromIso(iso_language);
        }
      }();

      // Setup topic
      std::stringstream pattern_ss;
      pattern_ss << "[ ";
      for (const auto& utterance: goal_handle->get_goal()->expected)
      {
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

      // auto memory_receiver = boost::make_shared<MemoryReceiver>();
      // auto service_id = session->registerService(topic_name, memory_receiver);
      auto memory = session->service("ALMemory").value();
      // memory.call<void>("subscribeToEvent", "Dialog/Answered", topic_name, "callback");
      auto memory_subscriber = memory.call<qi::AnyObject>("subscriber", topic_name + "/result");
      memory_subscriber.connect("signal", [=](const qi::AnyValue& value)
      {
        // This runs on a NAOqi callback thread: nothing may escape, or the
        // whole process terminates.
        try
        {
          auto utterance = value.toString();
          RCLCPP_INFO(state->logger, "Received input: %s", utterance.c_str());

          auto claimed = claim_goal(*state, goal_handle);
          if (!claimed.goal) {
            // Already cancelled (or not yet registered); drop the utterance.
            return;
          }
          claimed.memory_subscriber.reset();
          cleanup_dialog(*state, topic_name);

          result->result = std::vector<std::string>{utterance};
          goal_handle->succeed(result);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(state->logger, "Error while finishing listen goal: %s", e.what());
        }
        catch (...)
        {
          RCLCPP_ERROR(state->logger, "Unknown error while finishing listen goal");
        }
      });

      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->current_goal = goal_handle;
        state->memory_subscriber = memory_subscriber;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Failed to start listen %s: %s", goal_id.c_str(), e.what());
      auto claimed = claim_goal(*state, goal_handle);
      claimed.memory_subscriber.reset();
      cleanup_dialog(*state, topic_name);
      goal_handle->abort(result);
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<ListenState> state,
    const std::shared_ptr<ListenGoalHandle> goal_handle)
  {
    std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
    RCLCPP_INFO(state->logger, "Received goal cancellation request %s", goal_id.c_str());

    auto claimed = claim_goal(*state, goal_handle);
    if (!claimed.goal) {
      // The dialog callback already finished (or is finishing) this goal.
      return rclcpp_action::CancelResponse::REJECT;
    }

    claimed.memory_subscriber.reset();
    cleanup_dialog(*state, topic_name_for_goal_id(goal_handle->get_goal_id()));

    // The goal only enters CANCELING once we return ACCEPT, so the terminal
    // transition has to happen on another thread, after that.
    std::thread([state, goal_handle]
    {
      try
      {
        for (int i = 0; i < 100 && !goal_handle->is_canceling(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        goal_handle->canceled(std::make_shared<Listen::Result>());
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(
          state->logger, "Failed to mark listen %s as canceled: %s",
          rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(), e.what());
      }
    }).detach();

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void cleanup_dialog(ListenState& state, const std::string& topic_name)
  {
    auto& logger = state.logger;
    const auto& session = state.session;
    try
    {
      auto dialog = session->service("ALDialog").value();

      // try
      // {
      //   dialog.call<void>("unsubscribe", topic_name);
      // }
      // catch (const std::exception& e)
      // {
      //   RCLCPP_WARN(logger, "Failed to unsubscribe from topic %s: %s", topic_name.c_str(), e.what());
      // }

      try
      {
        dialog.call<void>("deactivateTopic", topic_name);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(logger, "Failed to deactivate topic %s: %s", topic_name.c_str(), e.what());
      }

      try
      {
        dialog.call<void>("unloadTopic", topic_name);
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(logger, "Failed to unload topic %s: %s", topic_name.c_str(), e.what());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "Failed to get ALDialog service while cleaning up: %s", e.what());
    }

    RCLCPP_INFO(logger, "Listen topic %s stopped and cleaned up", topic_name.c_str());
  }
}

rclcpp_action::Server<Listen>::SharedPtr 
createListenServer(rclcpp::Node* node, qi::SessionPtr session)
{
  namespace ph = std::placeholders;
  auto task = std::make_shared<ListenState>(node, std::move(session));
  return rclcpp_action::create_server<Listen>(
    node, "naoqi_driver/listen",
    std::bind(handle_goal, task, ph::_1, ph::_2),
    std::bind(handle_cancel, task, ph::_1),
    std::bind(handle_accepted, task, ph::_1)
  );
}

} // ends namespace action
} // ends namespace naoqi