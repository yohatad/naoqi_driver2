#include <memory>

#include "actions/goal_worker.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/listen.hpp>

namespace naoqi
{
namespace action
{

/** The listen action server; destroying the handle aborts the running goal. */
std::unique_ptr<ActionHandle>
createListenServer(rclcpp::Node * node, qi::SessionPtr session);

}  // namespace action
}  // namespace naoqi
