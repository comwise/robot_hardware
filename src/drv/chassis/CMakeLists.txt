cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(chassis STATIC ${SOURCE_FILES})
target_link_libraries(chassis motor curtis pthread rt)

add_executable(chassis_test chassis_test.cpp)
target_link_libraries(chassis_test chassis)