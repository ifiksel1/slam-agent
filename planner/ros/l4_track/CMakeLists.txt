cmake_minimum_required(VERSION 3.0.2)
project(l4_track)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# no -ffast-math: the planner relies on IEEE NaN / inf semantics as numpy did
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs quadrotor_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include third_party
  LIBRARIES l4_track_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs quadrotor_msgs
  DEPENDS EIGEN3
)

include_directories(include third_party ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# the planner core: ROS-free (local_ground is also the landing check's ground model)
add_library(l4_track_core STATIC src/track_core.cpp)
set_target_properties(l4_track_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(track_planner src/track_planner_node.cpp)
add_dependencies(track_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(track_planner l4_track_core ${catkin_LIBRARIES})

# the real-map gate (formerly planner/test/real_maps.py + test_track_planner.py --map/--gate)
add_executable(track_gate tools/track_gate.cpp)
target_link_libraries(track_gate l4_track_core)

install(TARGETS l4_track_core track_planner track_gate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/l4_track/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY third_party/nlohmann DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_track_core test/test_track_core.cpp)
  target_link_libraries(test_track_core l4_track_core)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_track_planner_node test/track_planner_node.test test/test_track_planner_node.cpp)
  target_link_libraries(test_track_planner_node ${catkin_LIBRARIES})
  add_dependencies(test_track_planner_node track_planner)
endif()
