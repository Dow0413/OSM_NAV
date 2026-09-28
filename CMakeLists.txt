cmake_minimum_required(VERSION 3.8)
project(osm_nav)

find_package(ament_cmake REQUIRED)

# Lanelet2 is vendored below src/lanelet2 and built as part of this one ROS
# package.  Its original module CMake files use find_package() between
# modules, so expose each generated local config before adding its dependents.
# mrt_cmake_modules classifies ROS dependencies by scanning AMENT_PREFIX_PATH;
# include the vendored source tree for this configure run so the internal
# lanelet2_* dependencies are recognised as local ROS packages.
set(ENV{AMENT_PREFIX_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lanelet2:$ENV{AMENT_PREFIX_PATH}")
macro(add_vendored_lanelet2_module module)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/lanelet2/${module}"
                   "lanelet2/${module}")
  set(${module}_DIR
      "${CMAKE_BINARY_DIR}/lanelet2/${module}/devel/share/${module}/cmake")
  list(PREPEND CMAKE_PREFIX_PATH
       "${CMAKE_BINARY_DIR}/lanelet2/${module}/devel")
endmacro()

add_vendored_lanelet2_module(lanelet2_core)
add_vendored_lanelet2_module(lanelet2_maps)
add_vendored_lanelet2_module(lanelet2_io)
add_vendored_lanelet2_module(lanelet2_projection)
add_vendored_lanelet2_module(lanelet2_traffic_rules)
add_vendored_lanelet2_module(lanelet2_routing)
add_vendored_lanelet2_module(lanelet2_matching)
add_vendored_lanelet2_module(lanelet2_validation)
add_vendored_lanelet2_module(lanelet2_python)
add_vendored_lanelet2_module(lanelet2_examples)

install(PROGRAMS
  scripts/osm_nav_server.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/pcd_overlay.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
