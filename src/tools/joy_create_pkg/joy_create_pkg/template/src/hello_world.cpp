#include <hello_world/hello_world.hpp>

#include <iostream>

namespace joypilot {
namespace module-name {
namespce pkg-name{

    int32_t print_hello() {
        std::cout << "Hello World" << std::endl;
        return 0;
    }

}  // namespace pkg-name
}  // namespace module-name
}  // namespace joypilot