cmake_minimum_required(VERSION 3.0.2)
project(l4_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp)

# header-only: include/l4_common (pyjson.h, params.h) and third_party/nlohmann (json.hpp, MIT)
catkin_package(
  INCLUDE_DIRS include third_party
  CATKIN_DEPENDS roscpp
)

include_directories(include third_party ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/l4_common/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY third_party/nlohmann DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_pyjson test/test_pyjson.cpp)
endif()
