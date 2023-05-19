
cmake_minimum_required(VERSION 3.2)

include_directories(log)

file(GLOB_RECURSE SOURCE_FILES "*.cpp")
add_library(logger STATIC ${SOURCE_FILES})
target_link_libraries(logger pthread rt)

add_executable(logger_test main.cpp)
target_link_libraries(logger_test logger)
