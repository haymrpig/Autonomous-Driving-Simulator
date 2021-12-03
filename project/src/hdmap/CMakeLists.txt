cmake_minimum_required(VERSION 3.0.2)
project(hdmap)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  parameter
  adasone_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp std_msgs parameter adasone_msgs sensor_msgs message_filters geometry_msgs cv_bridge
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(hdmap_node node/hdmap.cpp
                        src/Hdmap.cpp
)

target_link_libraries(hdmap_node
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

add_dependencies(hdmap_node ${adasone_msgs_EXPORTED_TARGETS}  ${coordinate_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


