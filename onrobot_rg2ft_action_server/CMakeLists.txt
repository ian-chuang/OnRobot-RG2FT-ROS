cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg2ft_action_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  onrobot_rg2ft_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  actionlib
  actionlib_msgs
  control_msgs
  onrobot_rg2ft_msgs
  roscpp
  sensor_msgs
)


include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(onrobot_rg2ft_action_server 
  src/onrobot_rg2ft_action_server_node.cpp 
  src/onrobot_rg2ft_action_server.cpp
)
add_dependencies(onrobot_rg2ft_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(onrobot_rg2ft_action_server ${catkin_LIBRARIES} )