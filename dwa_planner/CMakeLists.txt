cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  ros_adapter_node
)
find_package(Eigen3 REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_planner_lib
)



###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(dwa_planner_lib
  src/dwa_planner.cpp
  src/parameters.cpp
)
add_dependencies(dwa_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(dwa_planner src/dwa_planner_node.cpp)
target_link_libraries(dwa_planner
  ${catkin_LIBRARIES}
  dwa_planner_lib
)

install(TARGETS dwa_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
    )
install(TARGETS dwa_planner_lib
    LIBRARY DESTINATION lib          # Linux .so
    ARCHIVE DESTINATION lib          # Windows .lib / Linux .a
    RUNTIME DESTINATION bin          # Windows .dll
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
endif()
