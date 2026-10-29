cmake_minimum_required(VERSION 3.0.2)
project(l4_executive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs geometry_msgs mavros_msgs l4_common)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES l4_mission
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs mavros_msgs l4_common
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIR})

# the mission file (D6) and the mode numbers: ROS-free
add_library(l4_mission STATIC src/mission.cpp)
set_target_properties(l4_mission PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(l4_mission yaml-cpp)

add_executable(mission_executive src/mission_executive_node.cpp)
add_dependencies(mission_executive ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_executive l4_mission ${catkin_LIBRARIES} pthread)

install(TARGETS l4_mission mission_executive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/l4_executive/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  # the mission file and mode numbers
  catkin_add_gtest(test_mission test/test_mission.cpp)
  target_compile_definitions(test_mission PRIVATE MISSIONS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../missions")
  target_link_libraries(test_mission l4_mission)
  # the scenarios (formerly planner/test/test_mission_executive.py): the real executive as a child process
  # against a fake MAVROS / bridge / SUPER / track planner world
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_mission_executive_node test/mission_executive_node.test test/test_mission_executive_node.cpp)
  target_compile_definitions(test_mission_executive_node PRIVATE MISSION_EXECUTIVE="$<TARGET_FILE:mission_executive>")
  target_link_libraries(test_mission_executive_node ${catkin_LIBRARIES})
  add_dependencies(test_mission_executive_node mission_executive)
endif()
