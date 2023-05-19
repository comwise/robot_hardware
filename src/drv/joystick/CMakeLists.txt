cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(joystick STATIC ${SOURCE_FILES})
target_link_libraries(joystick pthread rt) 

add_executable(joy_test joy_test.cc)
target_link_libraries(joy_test joystick)