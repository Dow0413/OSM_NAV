cmake_minimum_required(VERSION 3.8)
project(galileo_osm_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/galileo_osm_nav_server.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY web config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
