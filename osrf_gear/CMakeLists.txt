cmake_minimum_required(VERSION 2.8.3)
project(osrf_gear)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  std_srvs
  message_generation
)

find_package(gazebo REQUIRED)

########################
## Message generation ##
########################

add_message_files(DIRECTORY msg
  FILES
  ConveyorBeltState.msg
)

add_service_files(DIRECTORY srv
  FILES
  ConveyorBeltControl.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libConveyorBeltPlugin.so library.
set(conveyor_belt_plugin_name ConveyorBeltPlugin)
add_library(${conveyor_belt_plugin_name} src/ConveyorBeltPlugin.cc)
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${side_contact_plugin_name}
)
install(TARGETS ${conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Create the libROSConveyorBeltPlugin.so library.
set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)
add_library(${ros_conveyor_belt_plugin_name} src/ROSConveyorBeltPlugin.cc)
target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(${ros_conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_conveyor_belt_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Create the libConveyorCongestionPlugin.so library.
# set(conveyor_congestion_plugin ConveyorCongestionPlugin)
# add_library(${conveyor_congestion_plugin} src/ConveyorCongestionPlugin.cc)
# target_link_libraries(${conveyor_congestion_plugin}
#   ${GAZEBO_LIBRARIES}
# )
# add_dependencies(${conveyor_congestion_plugin}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# install(TARGETS ${conveyor_congestion_plugin}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# # Create the libAriacTaskManagerPlugin.so library.
# set(task_plugin_name ROSAriacTaskManagerPlugin)
# add_library(${task_plugin_name}
#   src/ROSAriacTaskManagerPlugin.cc
# )
# target_link_libraries(${task_plugin_name}
#   ${GAZEBO_LIBRARIES}
#   ${roscpp_LIBRARIES}
#   AriacScorer
# )
# add_dependencies(${task_plugin_name}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
# )
# install(TARGETS ${task_plugin_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

#############
## Install ##
#############

# install(PROGRAMS script/gear.py script/gazebo_unpauser.py script/startup.sh
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# # Install all the config files
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
# 
# # Install all the world files
# install(DIRECTORY worlds/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
# 
# # Install all the model files
# install(DIRECTORY models/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
# 
# # Install all the vendor files from outside packages
# install(DIRECTORY vendor/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vendor)
