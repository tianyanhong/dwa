cmake_minimum_required(VERSION 3.0.2)
project(raycast_grid_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  tf
  tf2
  geometry_msgs
)
find_package(Eigen3 REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(grid_map_node
  src/grid_map_node.cpp
  src/grid_map.cpp
  src/ray_tracer.cpp
)

target_link_libraries(grid_map_node
  ${catkin_LIBRARIES}
  # ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS grid_map_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
    )
