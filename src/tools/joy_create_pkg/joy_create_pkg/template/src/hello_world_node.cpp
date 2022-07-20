#include <hello_world/hello_world_node.hpp>

namespace joypilot {
namespace module-name {
namespace pkg-name{

HelloWorldNode::HelloWorldNode(const rclcpp::NodeOptions & options)
:  Node("hello_world", options),
  verbose(true)
{
  print_hello();
}

int32_t HelloWorldNode::print_hello() const
{

  return hello_world::print_hello();
}
}  // namespace pkg-name
}  // namespace module-name
}  // namespace joypilot

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(joypilot::hello_world::HelloWorldNode)
