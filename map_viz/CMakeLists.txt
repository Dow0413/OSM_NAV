cmake_minimum_required(VERSION 3.8)
project(map_viz)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/osm_map_viz.py
  scripts/raw_osm_viz.py
  scripts/raw_osm_navigation_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
