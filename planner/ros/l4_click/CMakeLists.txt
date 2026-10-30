cmake_minimum_required(VERSION 3.0.2)
project(l4_click)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs l4_common)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES l4_click_core
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs l4_common
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# click -> goal (ROS-free)
add_library(l4_click_core STATIC src/click_goal.cpp)
set_target_properties(l4_click_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(click_goal src/click_goal_node.cpp)
add_dependencies(click_goal ${catkin_EXPORTED_TARGETS})
target_link_libraries(click_goal l4_click_core ${catkin_LIBRARIES})

install(TARGETS l4_click_core click_goal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/l4_click/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_click_goal test/test_click_goal.cpp)
  target_link_libraries(test_click_goal l4_click_core)
endif()
