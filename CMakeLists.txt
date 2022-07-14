cmake_minimum_required(VERSION 3.0.2)
project(ros1_rmf_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set (CMAKE_AUTOMOC ON)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_plugins
  std_srvs
  std_msgs
  message_generation
)


add_message_files(
   FILES
   DoorMode.msg
   DoorRequest.msg
   DoorState.msg
   LiftRequest.msg
   LiftState.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  gazebo_plugins
  std_srvs
  std_msgs
  message_generation
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
     ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})


include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rmf_sim_utils src/utils.cpp)
target_link_libraries(rmf_sim_utils ${GAZEBO_LIBRARIES})

add_library(toggle_floors SHARED ${headers_MOC} src/toggle_floors.cpp)
target_link_libraries(toggle_floors ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES})


add_library(gui_example_spawn_widget SHARED ${headers_MOC}
    src/GUIExampleSpawnWidget.cpp)
target_link_libraries(gui_example_spawn_widget ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES})



install(
  TARGETS
  gui_example_spawn_widget
  rmf_sim_utils
  toggle_floors
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
