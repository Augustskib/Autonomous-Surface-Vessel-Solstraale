cmake_minimum_required(VERSION 3.8)
project(boat_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionCommand.msg"
  "msg/ThrustCommand.msg"
  "msg/NavStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/PowerChannel.msg"
  "msg/PowerStatus.msg"
  "msg/HeadingEstimate.msg"
  "msg/FailsafeState.msg"
  "msg/CameraStatus.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
