cmake_minimum_required(VERSION 3.16)
project(ibvs_controller)

# Basic compiler warnings and options
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_kdl
  tf2_eigen
  tf2_eigen_kdl
  controller_interface
  geometry_msgs
  sensor_msgs
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  visual_servoing_controllers_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)

# Generate parameter library
find_package(generate_parameter_library REQUIRED)

# Generate parameter library
generate_parameter_library(ibvs_controller_params
  config/ibvs_controller_params.yaml
)

# Create the controller as a shared library
add_library(ibvs_controller SHARED
  src/ibvs_controller.cpp
)

# Specify the C++ standard
target_compile_features(ibvs_controller PUBLIC cxx_std_17)

# Include directories for header files
target_include_directories(ibvs_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against the necessary dependencies and parameter library
ament_target_dependencies(ibvs_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ibvs_controller PUBLIC ibvs_controller_params)

# Define the plugin for the controller
pluginlib_export_plugin_description_file(controller_interface ibvs_controller_plugin.xml)

# Install the shared library and headers
install(TARGETS
  ibvs_controller
  ibvs_controller_params
  EXPORT export_ibvs_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(export_ibvs_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()