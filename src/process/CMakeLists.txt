cmake_minimum_required(VERSION 3.2)

include_directories(process)

file(GLOB_RECURSE SOURCE_FILES "*.c*")
add_library(process STATIC subp_ctrler.cpp ros_ctrler.cpp)
#set_target_properties(process PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(process ros_node PocoFoundation tinyxml.a pthread rt)