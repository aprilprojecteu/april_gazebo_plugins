cmake_minimum_required(VERSION 3.5.1)
project(gazebo_ros_vel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  gazebo_ros_vel

  CATKIN_DEPENDS
  roscpp
  geometry_msgs
)

## Plugins
add_library(gazebo_ros_vel src/gazebo_ros_vel.cpp)
target_link_libraries(gazebo_ros_vel ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  gazebo_ros_vel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
