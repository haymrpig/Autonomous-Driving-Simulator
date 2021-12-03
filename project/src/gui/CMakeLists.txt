cmake_minimum_required(VERSION 3.0.2)
project(gui)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  adasone_msgs
  parameter
  message_filters
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs adasone_msgs parameter sensor_msgs message_filters
)

set(gui_INC_DIR include)

include_directories(
   include
   src
  ${catkin_INCLUDE_DIRS}
  ${gui_INC_DIR}
)

add_executable(gui_node node/guinode.cpp
                        src/driving.cpp
			src/gui.cpp
                        src/driving.ui)

target_link_libraries(gui_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  pthread
)
add_dependencies(gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
