cmake_minimum_required(VERSION 3.5)
project(metalimbs_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install urdf files
install(DIRECTORY robot/
  DESTINATION share/${PROJECT_NAME}/robot
)

# Install meshes
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install rviz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()