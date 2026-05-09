cmake_minimum_required(VERSION 3.0.2)
project(overtaking_iy)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  visualization_msgs
  f110_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs visualization_msgs f110_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/overtaking_iy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
