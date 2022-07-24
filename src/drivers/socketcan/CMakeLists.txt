cmake_minimum_required(VERSION 3.5)
project(socketcan)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/socket_can_common.hpp
    src/socket_can_common.cpp
    src/socket_can_id.cpp
    src/socket_can_receiver.cpp
    src/socket_can_sender.cpp)


ament_auto_package()