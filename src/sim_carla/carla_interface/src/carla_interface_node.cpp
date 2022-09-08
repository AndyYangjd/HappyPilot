#include <carla_interface/carla_interface_node.hpp>

namespace joypilot {
namespace module-name {
namespace pkg-name{

CarlaInterfaceNode::CarlaInterfaceNode(const rclcpp::NodeOptions & options)
:  Node("carla_interface", options),
  verbose(true)
{
  print_hello();
}

int32_t CarlaInterfaceNode::print_hello() const
{

  return carla_interface::print_hello();
}
}  // namespace pkg-name
}  // namespace module-name
}  // namespace joypilot

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(joypilot::carla_interface::CarlaInterfaceNode)
