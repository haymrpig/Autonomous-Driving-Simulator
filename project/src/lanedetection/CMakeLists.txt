cmake_minimum_required(VERSION 3.0.2)
project(lanedetection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  parameter
  adasone_msgs
  sensor_msgs
  OpenCV
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp std_msgs parameter adasone_msgs sensor_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lanedetection_node node/lanedetection.cpp
                        src/Detectlane.cpp
)

target_link_libraries(lanedetection_node
${catkin_LIBRARIES}
)

add_dependencies(lanedetection_node ${adasone_msgs_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



