cmake_minimum_required(VERSION 3.0.2)
project(l4_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# no -ffast-math: the core keeps numpy's IEEE arithmetic
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs mavros_msgs quadrotor_msgs l4_common)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES l4_bridge_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs mavros_msgs quadrotor_msgs l4_common
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# the core: ROS-free (frames, anchor, caps, geofence, HOLD logic, FC pose guard, SLAM health)
add_library(l4_bridge_core STATIC src/bridge_core.cpp)
set_target_properties(l4_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(planner_bridge src/planner_bridge_node.cpp src/planner_liveness.cpp)
add_dependencies(planner_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_bridge l4_bridge_core ${catkin_LIBRARIES} pthread)

install(TARGETS l4_bridge_core planner_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/l4_bridge/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(yaml-cpp REQUIRED)
  # the core tests (formerly planner/test/test_planner_bridge.py without --ros)
  catkin_add_gtest(test_bridge_core test/test_bridge_core.cpp)
  target_compile_definitions(test_bridge_core PRIVATE BRIDGE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../../config/planner_bridge.yaml")
  target_link_libraries(test_bridge_core l4_bridge_core yaml-cpp)
  # the node tests (formerly test_planner_bridge.py --ros): planner_bridge against synthetic inputs, and
  # planners in their own processes (fake_planner) killed -9
  find_package(rostest REQUIRED)
  add_executable(fake_planner test/fake_planner.cpp)
  add_dependencies(fake_planner ${catkin_EXPORTED_TARGETS})
  target_link_libraries(fake_planner ${catkin_LIBRARIES})
  add_rostest_gtest(test_planner_bridge_node test/planner_bridge_node.test test/test_planner_bridge_node.cpp)
  target_compile_definitions(test_planner_bridge_node PRIVATE FAKE_PLANNER="$<TARGET_FILE:fake_planner>")
  target_link_libraries(test_planner_bridge_node l4_bridge_core ${catkin_LIBRARIES})
  add_dependencies(test_planner_bridge_node planner_bridge fake_planner)
  add_rostest_gtest(test_bridge_yaw_node test/bridge_yaw_node.test test/test_bridge_yaw_node.cpp)
  target_link_libraries(test_bridge_yaw_node l4_bridge_core ${catkin_LIBRARIES} pthread)
  add_dependencies(test_bridge_yaw_node planner_bridge)
  add_rostest_gtest(test_bridge_turn_slow_node test/turn_slow_node.test test/test_bridge_turn_slow_node.cpp)
  target_link_libraries(test_bridge_turn_slow_node l4_bridge_core ${catkin_LIBRARIES} pthread)
  add_dependencies(test_bridge_turn_slow_node planner_bridge)
endif()
