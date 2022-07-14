cmake_minimum_required(VERSION 3.5)

project(joy_avp_demo)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(JOY_AVP_DEMO_LIB_SRC
  src/joy_avp_demo.cpp
)

set(JOY_AVP_DEMO_LIB_HEADERS
  include/joy_avp_demo/joy_avp_demo.hpp
  include/joy_avp_demo/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${JOY_AVP_DEMO_LIB_SRC}
  ${JOY_AVP_DEMO_LIB_HEADERS}
)


set(JOY_AVP_DEMO_NODE_SRC
  src/joy_avp_demo_node.cpp
)

set(JOY_AVP_DEMO_NODE_HEADERS
  include/joy_avp_demo/joy_avp_demo_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${JOY_AVP_DEMO_NODE_SRC}
  ${JOY_AVP_DEMO_NODE_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "joypilot::joy_avp_demo::JoyAvpDemoNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_joy_avp_demo.cpp)
  set(TEST_JOY_AVP_DEMO_EXE test_joy_avp_demo)
  ament_add_gtest(${TEST_JOY_AVP_DEMO_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_JOY_AVP_DEMO_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
