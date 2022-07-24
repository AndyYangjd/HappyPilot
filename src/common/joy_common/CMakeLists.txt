cmake_minimum_required(VERSION 3.5)

project(joy_common LANGUAGES "CXX")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install stuff
ament_auto_package()
