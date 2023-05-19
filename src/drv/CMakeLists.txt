cmake_minimum_required(VERSION 3.2)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/robot_hardware/drv
    ${PROJECT_SOURCE_DIR}/src/drv
)

add_subdirectory(modbus)
add_subdirectory(serial)
add_subdirectory(motor)
add_subdirectory(chassis)
add_subdirectory(joystick)
add_subdirectory(adapter)

file(GLOB_RECURSE SOURCE_FILES "*.c*")
add_library(drv
    STATIC
    drv_io.cpp
    drv_impl.cpp
    drv_subp.cpp
)
target_link_libraries(
    drv
    adapter
    etc
    ros_node
    process
    net
    serial
)

add_executable(drv_test drv_test.cpp)
target_link_libraries(drv_test drv)