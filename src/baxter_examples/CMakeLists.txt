cmake_minimum_required(VERSION 2.8.3)
project(baxter_examples)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/JointSpringsExample.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  xacro
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface
)

catkiN_install_python(
  PROGRAMS 
  scripts/analog_io_rampup.py
scripts/digital_io_blink.py
scripts/gripper_action_client.py
scripts/gripper_cuff_control.py
scripts/gripper_joystick.py
scripts/gripper_keyboard.py
scripts/head_action_client.py
scripts/head_wobbler.py
scripts/ik_service_client.py
scripts/joint_position_file_playback.py
scripts/joint_position_joystick.py
scripts/joint_position_keyboard.py
scripts/joint_position_waypoints.py
scripts/joint_recorder.py
scripts/joint_torque_springs.py
scripts/joint_trajectory_client.py
scripts/joint_trajectory_file_playback.py
scripts/joint_velocity_puppet.py
scripts/joint_velocity_wobbler.py
scripts/navigator_io.py
scripts/send_urdf_fragment.py
scripts/xdisplay_image.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
