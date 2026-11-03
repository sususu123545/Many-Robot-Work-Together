cmake_minimum_required(VERSION 3.8)
project(vla_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VlaAction.msg"
  "msg/VlaActionChunk.msg"
  "msg/VlaObservation.msg"
  "msg/VlaSafetyVerdict.msg"
  "srv/GetPolicyInfo.srv"
  "action/ExecuteActionChunk.action"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
