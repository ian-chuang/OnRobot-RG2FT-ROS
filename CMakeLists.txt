cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2ft_ros)
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
  FILES
  RG2FTCommand.msg
  RG2FTState.msg
) 

add_service_files(
  FILES
  SetProximityOffsets.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
  CATKIN_DEPENDS
  rospy
)

catkin_install_python(
  PROGRAMS
  scripts/OnRobotRG2FTDriver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
