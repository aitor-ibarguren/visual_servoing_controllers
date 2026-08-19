cmake_minimum_required(VERSION 3.8)
project(visual_servoing_controllers_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "action/PositionBasedVisualServoing.action"
    "action/ImageBasedVisualServoing.action"
    DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
