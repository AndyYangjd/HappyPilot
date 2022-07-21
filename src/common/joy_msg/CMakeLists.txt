cmake_minimum_required(VERSION 3.5)

project(joy_msg LANGUAGES "CXX")

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
    # Helper types
    "msg/BoundingBox.idl"
    "msg/Complex32.idl"
    "msg/DiagnosticHeader.idl"
    "msg/Quaternion32.idl"
    "msg/TrajectoryPoint.idl"
    "msg/MapPrimitive.idl"
    # Interfaces
    "msg/BoundingBoxArray.idl"
    "msg/ControlDiagnostic.idl"
    "msg/HighLevelControlCommand.idl"
    "msg/RawControlCommand.idl"
    "msg/Trajectory.idl"
    "msg/VehicleControlCommand.idl"
    "msg/VehicleKinematicState.idl"
    "msg/VehicleOdometry.idl"
    "msg/VehicleStateCommand.idl"
    "msg/VehicleStateReport.idl"
    "msg/HADMapBin.idl"
    "msg/Route.idl"
    "srv/AutonomyModeChange.idl"
    "srv/HADMapService.idl"
    "srv/ModifyTrajectory.idl"
    "action/PlanTrajectory.idl"
    # Implementation-specific messages
    "msg/PointClusters.idl"
    DEPENDENCIES "std_msgs" "geometry_msgs" "sensor_msgs"
    ADD_LINTER_TESTS
    )

# Install stuff
ament_auto_package()
