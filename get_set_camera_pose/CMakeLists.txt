cmake_minimum_required(VERSION 3.5.1)
project(get_set_camera_pose)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  std_srvs
)

find_package(gazebo REQUIRED)

# Add your service file here (SetCameraPose.srv is an example name)
add_service_files(
  FILES
  SetCameraPose.srv
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  get_set_camera_pose

  CATKIN_DEPENDS
  roscpp
)

## Plugins
add_library(get_set_camera_pose src/get_set_camera_pose.cpp)
target_link_libraries(get_set_camera_pose ${catkin_LIBRARIES})

install(TARGETS
  get_set_camera_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
