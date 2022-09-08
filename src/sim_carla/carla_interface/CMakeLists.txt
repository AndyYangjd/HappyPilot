cmake_minimum_required(VERSION 3.5)

project(carla_interface LANGUAGES "CXX")

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CARLA_INTERFACE_LIB_SRC
  src/carla_interface.cpp
)

set(CARLA_INTERFACE_LIB_HEADERS
  include/carla_interface/carla_interface.hpp
  include/carla_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${CARLA_INTERFACE_LIB_SRC}
  ${CARLA_INTERFACE_LIB_HEADERS}
)


set(CARLA_INTERFACE_NODE_SRC
  src/carla_interface_node.cpp
)

set(CARLA_INTERFACE_NODE_HEADERS
  include/carla_interface/carla_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${CARLA_INTERFACE_NODE_SRC}
  ${CARLA_INTERFACE_NODE_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "joypilot::carla_interface::CarlaInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_carla_interface.cpp)
  set(TEST_CARLA_INTERFACE_EXE test_carla_interface)
  ament_add_gtest(${TEST_CARLA_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_CARLA_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# Install stuff
ament_auto_package(INSTALL_TO_SHARE
  launch
)
