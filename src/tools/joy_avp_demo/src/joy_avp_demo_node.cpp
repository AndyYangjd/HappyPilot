#include "joy_avp_demo/joy_avp_demo_node.hpp"

namespace joypilot
{
namespace joy_avp_demo
{

JoyAvpDemoNode::JoyAvpDemoNode(const rclcpp::NodeOptions & options)
:  Node("joy_avp_demo", options),
  verbose(true)
{
  print_hello();
}

int32_t JoyAvpDemoNode::print_hello() const
{
  return joy_avp_demo::print_hello();
}

}  // namespace joy_avp_demo
}  // namespace joypilot

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(joypilot::joy_avp_demo::JoyAvpDemoNode)
