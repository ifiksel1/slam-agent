cmake_minimum_required(VERSION 3.0.2)
project(l4_sim_viz)

# SIM ONLY (click and run): the run's map, /tf world -> drone and the robot's box for a live 3D view

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2_ros)
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2_ros)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sim_viz src/sim_viz_node.cpp)
target_link_libraries(sim_viz ${catkin_LIBRARIES})
install(TARGETS sim_viz RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
