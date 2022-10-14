cmake_minimum_required(VERSION 3.14)
project(testframework)
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf)
find_package(rostest REQUIRED)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(twoDrones_test src/twoDrones_test.cpp)
target_link_libraries(twoDrones_test ${catkin_LIBRARIES})

