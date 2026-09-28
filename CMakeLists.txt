cmake_minimum_required(VERSION 3.8)
project(osm_nav)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/osm_nav_server.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/pcd_overlay.py DESTINATION lib/${PROJECT_NAME})
# Python algorithm modules are installed next to the executable so it can be
# launched from either a source checkout or an installed ROS workspace.
install(DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}/src
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY web config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
