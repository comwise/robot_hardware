cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(serial STATIC ${SOURCE_FILES})
target_link_libraries(serial serial_sdk pthread rt) 

add_executable(serial_test serial_test.cc)
target_link_libraries(serial_test serial) 