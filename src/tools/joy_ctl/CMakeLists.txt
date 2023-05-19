cmake_minimum_required(VERSION 3.2)

add_compile_options(-std=c++11 -g)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(joy_ctl STATIC ${SOURCE_FILES})
target_link_libraries(joy_ctl ros_node pthread rt)

add_executable(robot_joystick joy_node.cc)
target_link_libraries(robot_joystick joy_ctl)