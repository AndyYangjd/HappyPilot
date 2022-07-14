#ifndef HELLO_WORLD__HELLO_WORLD_NODE_HPP_
#define HELLO_WORLD__HELLO_WORLD_NODE_HPP_


#include "hello_world/hello_world.hpp"

#include <rclcpp/rclcpp.hpp>


namespace joypilot
{
namespace hello_world
{

class HelloWorldNode final : public rclcpp::Node
{
public:
    HelloWorldNode() = default;
    ~HelloWorldNode() = default;

    explicit HelloWorldNode(const rclcpp::NodeOptions & options);

    int32_t print_hello() const;

private:
    bool verbose;
};
}  // namespace hello_world
}  // namespace joypilot

#endif  // HELLO_WORLD__HELLO_WORLD_NODE_HPP_
