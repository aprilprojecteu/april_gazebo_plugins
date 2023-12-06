cmake_minimum_required(VERSION 3.5.1)
project(get_set_camera_pose)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  geometry_msgs
  message_generation
  roscpp
  std_srvs
)

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
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    gazebo_dev
    geometry_msgs
    message_runtime
    roscpp
    std_srvs
)

## Plugins
add_library(${PROJECT_NAME} src/get_set_camera_pose.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
