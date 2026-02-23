cmake_minimum_required(VERSION 3.8)
project(aurora_sdk_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# Install Python executable
install(PROGRAMS
  scripts/aurora_sdk_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aurora_sdk_bridge_node
)

# Install config and launch
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
