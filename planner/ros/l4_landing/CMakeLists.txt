cmake_minimum_required(VERSION 3.0.2)
project(l4_landing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS l4_track roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES l4_landing_core
  CATKIN_DEPENDS l4_track roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# the check (ROS-free) and the marker helpers (ROS messages, no node)
add_library(l4_landing_core STATIC src/landing_core.cpp src/landing_markers.cpp)
set_target_properties(l4_landing_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_dependencies(l4_landing_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(l4_landing_core ${catkin_LIBRARIES})

add_executable(landing_check src/landing_check_node.cpp)
target_link_libraries(landing_check l4_landing_core ${catkin_LIBRARIES})

install(TARGETS l4_landing_core landing_check
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/l4_landing/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_landing_core test/test_landing_core.cpp)
  target_link_libraries(test_landing_core l4_landing_core ${catkin_LIBRARIES})
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_landing_check_node test/landing_check_node.test test/test_landing_check_node.cpp)
  target_link_libraries(test_landing_check_node ${catkin_LIBRARIES})
  add_dependencies(test_landing_check_node landing_check)
endif()
