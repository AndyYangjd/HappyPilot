project(carla_ad_demo)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

ament_package()

