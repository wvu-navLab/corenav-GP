cmake_minimum_required(VERSION 2.8.3)
project(init_core_nav)

set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_utils
  parameter_utils
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf
  roslib
  novatel_gps_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_utils
    parameter_utils
    geometry_msgs
    sensor_msgs
    tf2_ros
    tf
    roslib
    novatel_gps_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/InitCoreNav.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
