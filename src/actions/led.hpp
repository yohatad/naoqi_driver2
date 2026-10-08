#include <memory>

#include "actions/goal_worker.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/run_led.hpp>

namespace naoqi
{
namespace action
{

/** The LED action server; destroying the handle stops the running goal. */
std::unique_ptr<ActionHandle>
createRunLedServer(rclcpp::Node * node, qi::SessionPtr session);

}  // namespace action
}  // namespace naoqi
