cmake_minimum_required(VERSION 3.0.2)
project(mkmr_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mkmr_description
  controller_manager
  joint_state_controller
  position_controllers
  velocity_controllers
  controller_interface
  realtime_tools
)

set(mecanum_drive_controller_sources
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mecanum_drive_controller
  CATKIN_DEPENDS controller_interface realtime_tools
)


###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(mecanum_drive_controller ${mecanum_drive_controller_headers} ${mecanum_drive_controller_sources})
target_link_libraries(mecanum_drive_controller ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS mecanum_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES mecanum_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

