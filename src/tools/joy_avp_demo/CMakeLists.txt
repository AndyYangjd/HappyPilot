cmake_minimum_required(VERSION 3.5)

project(joy_avp_demo)

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

# Testing
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# Install stuff
ament_auto_package(INSTALL_TO_SHARE
	data
	config
	param
	launch
)
