cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_ultrasonic_demo_gazebo)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY world rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
