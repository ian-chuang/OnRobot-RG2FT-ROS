cmake_minimum_required(VERSION 2.8.3)
project(onrobot_rg2ft_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  FILES
  RG2FTCommand.msg
  RG2FTState.msg
) 

add_service_files(
  FILES
  SetProximityOffsets.srv
)

generate_messages(DEPENDENCIES)
catkin_package(CATKIN_DEPENDS message_runtime)

                      
