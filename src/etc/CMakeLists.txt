cmake_minimum_required(VERSION 3.2)

include_directories(/usr/include/jsoncpp etc)

file(GLOB SOURCE_FILES "*.cpp")

add_library(etc STATIC ${SOURCE_FILES})
target_link_libraries(etc jsoncpp.a pthread)

add_executable(etc_test etc_test.cpp)
target_link_libraries(etc_test etc) 