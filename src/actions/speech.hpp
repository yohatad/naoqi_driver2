#ifndef SPEECH_ACTION_HPP
#define SPEECH_ACTION_HPP

#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/speech_with_feedback.hpp>

namespace naoqi
{
namespace action {

rclcpp_action::Server<naoqi_bridge_msgs::action::SpeechWithFeedback>::SharedPtr
createSpeechWithFeedbackServer(rclcpp::Node* node, qi::SessionPtr session);

} // namespace action
} // namespace naoqi

#endif // SPEECH_ACTION_HPP