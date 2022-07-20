#ifndef MODULE_NAME__PKG_NAME__FILE_NAME_HPP_
#define MODULE_NAME__PKG_NAME__FILE_NAME_HPP_


#include <hello_world/hello_world.hpp>

#include <rclcpp/rclcpp.hpp>


namespace joypilot {
namespace module-name {
namespace pkg-name {

class HelloWorldNode final : public rclcpp::Node{
    public:
        HelloWorldNode() = default;
        ~HelloWorldNode() = default;

        explicit HelloWorldNode(const rclcpp::NodeOptions & options);

        int32_t print_hello() const;

    private:
        bool verbose;
};

}  // namespace pkg-name
}  // namespace module-name
}  // namespace joypilot

#endif  // MODULE_NAME__PKG_NAME__FILE_NAME_HPP_
