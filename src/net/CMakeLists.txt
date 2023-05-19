cmake_minimum_required(VERSION 3.2)

include_directories(net)

add_library(net_utils STATIC net_utils.cpp)
set_target_properties(net_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(net_utils PocoFoundation cap.a)

file(GLOB_RECURSE SOURCE_FILES "*.c*")
add_library(net STATIC net_utils.cpp net_plugin.cpp)
target_link_libraries(net PocoFoundation cap.a)

add_executable(net_test main.cpp)
target_link_libraries(net_test net)