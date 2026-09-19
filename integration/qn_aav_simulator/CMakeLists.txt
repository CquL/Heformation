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
  std_srvs
)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES PlatformSegment.msg)
add_service_files(DIRECTORY srv FILES TakeReference.srv)
add_action_files(DIRECTORY action FILES Formation.action PlatformTask.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS actionlib_msgs diagnostic_msgs geometry_msgs message_runtime nav_msgs quadrotor_msgs rospy sensor_msgs std_msgs std_srvs)

catkin_install_python(
  PROGRAMS scripts/qn_aav_node.py scripts/formation_action_server.py
  scripts/formation_mission_runner.py scripts/verify_formation_experiment.py
  scripts/plot_formation_experiment.py scripts/plot_mission_overview.py
  scripts/animate_mission_replay.py scripts/mission_dashboard.py
  scripts/scene_publisher.py
  scripts/locate_failure_layer.py
  scripts/probe_safety_hold.py
  scripts/pvs_node.py scripts/measure_qn_modes.py scripts/probe_platform_fragment.py
  scripts/mission_console.py scripts/probe_pvs_actions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
