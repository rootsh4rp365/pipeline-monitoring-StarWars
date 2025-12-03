cmake_minimum_required(VERSION 3.0.2)
project(pipeline_monitoring_starwars)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs mavros_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/pipeline_scanner.py
  src/generate_world.py
  web/web.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map
)
