cmake_minimum_required(VERSION 2.8.3)
project(rsj_pick_and_place)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
  control_msgs
  )

install(
  PROGRAMS src/pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
