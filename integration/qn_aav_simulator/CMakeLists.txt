cmake_minimum_required(VERSION 3.0.2)
project(qn_aav_simulator)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_action_files(DIRECTORY action FILES Formation.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS actionlib_msgs diagnostic_msgs geometry_msgs message_runtime nav_msgs quadrotor_msgs rospy sensor_msgs std_msgs)

catkin_install_python(
  PROGRAMS scripts/qn_aav_node.py scripts/formation_action_server.py scripts/formation_mission_runner.py scripts/verify_formation_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
