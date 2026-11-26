cmake_minimum_required(VERSION 3.8)
project(handeye_autopilot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(service_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GateResult.msg"
  "msg/QualityReport.msg"
  "action/Calibrate.action"
  DEPENDENCIES geometry_msgs action_msgs service_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
