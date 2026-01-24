#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/run_led.hpp>

namespace naoqi
{
namespace action {

rclcpp_action::Server<naoqi_bridge_msgs::action::RunLed>::SharedPtr
createRunLedServer(rclcpp::Node* node, qi::SessionPtr session);

} // namespace action
} // namespace naoqi
