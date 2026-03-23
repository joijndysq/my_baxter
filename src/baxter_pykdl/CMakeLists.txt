cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()
project(baxter_pykdl)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  baxter_core_msgs
  baxter_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  baxter_interface
  baxter_core_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
