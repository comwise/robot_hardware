cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(adapter STATIC ${SOURCE_FILES})
target_link_libraries(adapter etc chassis motor joystick pthread rt)

add_executable(adapter_test adapter_test.cc)
target_link_libraries(adapter_test adapter)