cmake_minimum_required(VERSION 3.5)
project(metalimbs_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-attributes)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)
find_package(metalimbs_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Include directories
include_directories(include)

# Build library
add_library(${PROJECT_NAME}_lib
  src/metalimbs_hardware/hardware.cpp
  src/metalimbs_hardware/serial.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  std_srvs
  urdf
  metalimbs_srvs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# Build executables
add_executable(metalimbs_hardware_node
  src/metalimbs_hardware/metalimbs_hardware_node.cpp
)

target_link_libraries(metalimbs_hardware_node ${PROJECT_NAME}_lib)

ament_target_dependencies(metalimbs_hardware_node
  rclcpp
  sensor_msgs
  std_srvs
  urdf
  metalimbs_srvs
)

add_executable(arm_control_service_node
  src/metalimbs_hardware/arm_control_service_node.cpp
  src/metalimbs_hardware/arm_control_service.cpp
)

target_link_libraries(arm_control_service_node ${PROJECT_NAME}_lib)

ament_target_dependencies(arm_control_service_node
  rclcpp
  sensor_msgs
  std_srvs
  urdf
  metalimbs_srvs
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  metalimbs_hardware_node
  arm_control_service_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_srvs
  urdf
  metalimbs_srvs
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()