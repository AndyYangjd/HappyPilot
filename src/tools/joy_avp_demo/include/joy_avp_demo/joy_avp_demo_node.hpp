#ifndef JOY_AVP_DEMO__JOY_AVP_DEMO_NODE_HPP_
#define JOY_AVP_DEMO__JOY_AVP_DEMO_NODE_HPP_


#include "joy_avp_demo/joy_avp_demo.hpp"

#include <rclcpp/rclcpp.hpp>


namespace joypilot
{
namespace joy_avp_demo
{

class JoyAvpDemoNode final : public rclcpp::Node
{
public:
    JoyAvpDemoNode() = default;
    ~JoyAvpDemoNode() = default;

    explicit JoyAvpDemoNode(const rclcpp::NodeOptions & options);

    int32_t print_hello() const;

private:
    bool verbose;
};
}  // namespace joy_avp_demo
}  // namespace joypilot

#endif  // JOY_AVP_DEMO__JOY_AVP_DEMO_NODE_HPP_
