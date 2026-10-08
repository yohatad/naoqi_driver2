#ifndef SPEECH_ACTION_HPP
#define SPEECH_ACTION_HPP

#include <memory>

#include "actions/goal_worker.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/speech_with_feedback.hpp>

namespace naoqi
{
namespace action
{

/** The speech action server; destroying the handle stops the running goal. */
std::unique_ptr<ActionHandle>
createSpeechWithFeedbackServer(rclcpp::Node * node, qi::SessionPtr session);

}  // namespace action
}  // namespace naoqi

#endif // SPEECH_ACTION_HPP
