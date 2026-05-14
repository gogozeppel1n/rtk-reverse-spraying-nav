cmake_minimum_required(VERSION 3.0.2)
project(rtk_reverse_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf
  novatel_oem7_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/inspvax_to_utm_pose.py
  scripts/reverse_track_nav_node.py
  scripts/controller_panel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
