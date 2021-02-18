cmake_minimum_required(VERSION 2.8.3)
project(core_nav)

set (CMAKE_CXX_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  hw_interface_plugin_roboteq
  roscpp
  geometry_utils
  parameter_utils
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf
  std_msgs
  message_generation
  libcluster
)

add_message_files(
    FILES
    GP_Input.msg
    GP_Output.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
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
    libcluster
    EigenRand
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

file(GLOB SOURCES "src/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  #${PROJECT_NAME} /home/ck/corenav_dev/src/corenav-GP/libcluster/lib/libcluster.so
  ${PROJECT_NAME} /home/navlab-shounak/robust_ws/src/corenav-GP/libcluster/lib/libcluster.so

)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} $(catkin_EXPORTED_TARGETS))
