cmake_minimum_required(VERSION 3.0.2)
project(ros_manipulator_hit_the_ball)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  ball_track_alvar_msgs
  open_manipulator_msgs
  roscpp
  sensor_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    ball_track_alvar_msgs 
    open_manipulator_msgs 
    roscpp 
    sensor_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_manipulator_hit_the_ball src/ros_manipulator_hit_the_ball.cpp)
add_dependencies(ros_manipulator_hit_the_ball ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_manipulator_hit_the_ball ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS ros_manipulator_hit_the_ball
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############
