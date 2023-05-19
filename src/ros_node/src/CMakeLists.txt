cmake_minimum_required(VERSION 3.2)

add_library(ros_node STATIC ${CMAKE_CURRENT_SOURCE_DIR}/ros_node.cpp)
set_target_properties(ros_node PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ros_node ${catkin_LIBRARIES})
target_include_directories(ros_node PUBLIC ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)