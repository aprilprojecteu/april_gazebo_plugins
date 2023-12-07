cmake_minimum_required(VERSION 3.5.1)
project(conveyor_belt_sim)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  roscpp
  std_msgs
  std_srvs
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


###################################
## catkin specific configuration ##
###################################

set(conveyor_belt_plugin_name ConveyorBeltPlugin)
set(ros_conveyor_belt_plugin_name ROSConveyorBeltPlugin)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${conveyor_belt_plugin_name}
    ${ros_conveyor_belt_plugin_name}
  CATKIN_DEPENDS
    gazebo_ros
    message_runtime
    roscpp
    std_msgs
    std_srvs
  DEPENDS
    GAZEBO
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libConveyorBeltPlugin.so library.
add_library(${conveyor_belt_plugin_name} src/ConveyorBeltPlugin.cc)
target_link_libraries(${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Create the libROSConveyorBeltPlugin.so library.
add_library(${ros_conveyor_belt_plugin_name} src/ROSConveyorBeltPlugin.cc)
target_link_libraries(${ros_conveyor_belt_plugin_name}
  ${conveyor_belt_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${ros_conveyor_belt_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${conveyor_belt_plugin_name} ${ros_conveyor_belt_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
)
