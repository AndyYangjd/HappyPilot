cmake_minimum_required(VERSION 3.5)

project(hello_world LANGUAGES "CXX")

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(HELLO_WORLD_LIB_SRC
  src/hello_world.cpp
)

set(HELLO_WORLD_LIB_HEADERS
  include/hello_world/hello_world.hpp
  include/hello_world/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${HELLO_WORLD_LIB_SRC}
  ${HELLO_WORLD_LIB_HEADERS}
)


set(HELLO_WORLD_NODE_SRC
  src/hello_world_node.cpp
)

set(HELLO_WORLD_NODE_HEADERS
  include/hello_world/hello_world_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${HELLO_WORLD_NODE_SRC}
  ${HELLO_WORLD_NODE_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "joypilot::hello_world::HelloWorldNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_hello_world.cpp)
  set(TEST_HELLO_WORLD_EXE test_hello_world)
  ament_add_gtest(${TEST_HELLO_WORLD_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_HELLO_WORLD_EXE} ${PROJECT_NAME})
endif()

# Install stuff
ament_auto_package(INSTALL_TO_SHARE
  launch
)
