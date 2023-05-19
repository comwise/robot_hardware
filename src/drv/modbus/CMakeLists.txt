cmake_minimum_required(VERSION 3.2)

file(GLOB_RECURSE SOURCE_FILES "*.c*")

add_library(modbus STATIC ${SOURCE_FILES})
target_link_libraries(modbus modbus_sdk pthread rt) 

add_executable(modbus_test modbus_test.cc)
target_link_libraries(modbus_test modbus) 