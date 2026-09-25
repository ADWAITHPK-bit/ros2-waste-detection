cmake_minimum_required(VERSION 3.8)
project(sar_database_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WasteTarget.msg"
  DEPENDENCIES
    geometry_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()