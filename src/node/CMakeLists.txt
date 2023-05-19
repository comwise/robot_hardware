cmake_minimum_required(VERSION 3.2)

include_directories(node)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(node STATIC ${SOURCE_FILES})
target_link_libraries(node srv pthread rt)

add_executable(robot_hardware main.cpp)
target_link_libraries(robot_hardware node)