cmake_minimum_required(VERSION 3.16)
project(roborun_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteProgram.action"
  "msg/AlarmState.msg"
  "msg/AlarmStateEntry.msg"
  "msg/IoState.msg"
  "msg/RuntimeStatus.msg"
  "msg/SignalState.msg"
  "msg/ToolState.msg"
  "srv/ControlRuntime.srv"
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
