cmake_minimum_required(VERSION 3.2)
project(ros_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  diagnostic_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  move_cmd.msg
  move_feedback.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  std_api.srv
  std_json.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES ros_node
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
