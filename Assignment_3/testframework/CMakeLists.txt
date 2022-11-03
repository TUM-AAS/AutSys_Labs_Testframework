cmake_minimum_required(VERSION 3.14)
project(testframework)
set(CMAKE_CXX_STANDARD 11)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  mav_msgs
  eigen_conversions
  mav_planning_msgs
  tf_conversions
  eigen_conversions
)


catkin_package(CATKIN_DEPENDS 
  roscpp  
  mav_msgs
  mav_planning_msgs
  tf_conversions
  eigen_conversions
)

find_package(Eigen3)
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(trajectory_test src/trajectory_test.cpp)
target_link_libraries(trajectory_test ${catkin_LIBRARIES})
