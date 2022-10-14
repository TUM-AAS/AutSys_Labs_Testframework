cmake_minimum_required(VERSION 3.4)
project(testing)

# Setup testing
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

# Add test cpp file
add_executable(runTest
    testing.cpp
)

# Link test executable against gtest & gtest_main
target_link_libraries(runTest ${GTEST_LIBRARIES} pthread)

add_test(
    NAME runTest
    COMMAND runTest
)
