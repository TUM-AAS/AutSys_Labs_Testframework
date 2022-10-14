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

add_executable(controller_test src/controller_test.cpp)
target_link_libraries(controller_test ${catkin_LIBRARIES})

