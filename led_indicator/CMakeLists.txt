cmake_minimum_required(VERSION 3.5.1)
project(led_indicator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
)

find_package(gazebo REQUIRED)
find_package(gazebo_plugins REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
#   ${roscpp_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libLedPluginM.so library.
set(plugin_name LedPluginM)
add_library(${plugin_name} src/LedPluginM.cc)
target_link_libraries(${plugin_name} ${GAZEBO_LIBRARIES} /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libFlashLightPlugin.so)
install(TARGETS ${plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
