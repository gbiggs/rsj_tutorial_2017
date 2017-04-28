cmake_minimum_required(VERSION 2.8.3)
project(rsj_pick_and_place)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
)

find_package(
  Boost REQUIRED
  system
  filesystem
  date_time
  thread
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    control_msgs
    moveit_core
    moveit_ros_planning_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(pick_and_place
  src/pick_and_place.cpp
)
target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
install(
  TARGETS pick_and_place
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS src/pick_and_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
