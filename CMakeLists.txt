cmake_minimum_required(VERSION 2.8.3)
project(kaqi_teleop)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  topic_tools
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kaqi_teleop
#  CATKIN_DEPENDS actionlib geometry_msgs roscpp rospy std_msgs tf topic_tools
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_kaqi src/teleop_kaqi.cpp)
add_executable(teleop_kaqi_keyboard src/teleop_kaqi_keyboard.cpp)

target_link_libraries(teleop_kaqi
  ${catkin_LIBRARIES}
)
target_link_libraries(teleop_kaqi_keyboard
  ${catkin_LIBRARIES}
)
