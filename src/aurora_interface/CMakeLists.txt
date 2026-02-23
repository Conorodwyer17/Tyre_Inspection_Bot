cmake_minimum_required(VERSION 3.8)
project(aurora_interface)

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/aurora_health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aurora_health_monitor
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
