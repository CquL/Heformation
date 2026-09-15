cmake_minimum_required(VERSION 3.0.2)
project(qn_aav_simulator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(
  PROGRAMS scripts/qn_aav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
