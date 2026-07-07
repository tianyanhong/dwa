cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
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

add_executable(path_searching
)

target_link_libraries(path_searching
  ${catkin_LIBRARIES}
  # ${EIGEN3_INCLUDE_DIRS}
)