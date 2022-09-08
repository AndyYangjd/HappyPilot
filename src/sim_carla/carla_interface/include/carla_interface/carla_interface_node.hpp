#ifndef MODULE_NAME__PKG_NAME__FILE_NAME_HPP_
#define MODULE_NAME__PKG_NAME__FILE_NAME_HPP_


#include <carla_interface/carla_interface.hpp>

#include <rclcpp/rclcpp.hpp>


namespace joypilot {
namespace module-name {
namespace pkg-name {

class CarlaInterfaceNode final : public rclcpp::Node{
    public:
        CarlaInterfaceNode() = default;
        ~CarlaInterfaceNode() = default;

        explicit CarlaInterfaceNode(const rclcpp::NodeOptions & options);

        int32_t print_hello() const;

    private:
        bool verbose;
};

}  // namespace pkg-name
}  // namespace module-name
}  // namespace joypilot

#endif  // MODULE_NAME__PKG_NAME__FILE_NAME_HPP_
