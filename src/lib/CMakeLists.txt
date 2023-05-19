cmake_minimum_required(VERSION 3.2)

include_directories(${PROJECT_SOURCE_DIR}/include/robot_hardware/lib)

add_subdirectory(modbus)
add_subdirectory(serial)
#add_subdirectory(spdlog)