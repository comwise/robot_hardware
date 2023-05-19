cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(motor STATIC ${SOURCE_FILES})
target_link_libraries(motor can pthread rt) 

add_executable(motor_test motor_test.cc)
target_link_libraries(motor_test motor)