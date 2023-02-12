cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2ft_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  onrobot_rg2ft_msgs
  std_srvs
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  CATKIN_DEPENDS
  rospy
  onrobot_rg2ft_msgs
  std_srvs
)

catkin_install_python(
  PROGRAMS
  scripts/OnRobotRG2FTDriver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
