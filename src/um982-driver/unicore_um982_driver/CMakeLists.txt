cmake_minimum_required(VERSION 3.0.2)
project(unicore_um982_driver)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  roscpp
  serial
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs roscpp serial sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(unicore_um982_driver_node
  src/unicore_um982_driver_node.cpp
  src/pvtsln_parser.cpp
)

target_compile_features(unicore_um982_driver_node PRIVATE cxx_std_14)
target_link_libraries(unicore_um982_driver_node ${catkin_LIBRARIES})
add_dependencies(unicore_um982_driver_node ${catkin_EXPORTED_TARGETS})

install(TARGETS unicore_um982_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
