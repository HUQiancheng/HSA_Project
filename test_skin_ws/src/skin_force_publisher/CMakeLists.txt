cmake_minimum_required(VERSION 3.0.2)
project(skin_force_publisher)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tum_ics_skin_msgs
  tum_ics_skin_bridge
  tum_ics_skin_descr
)

## Find Eigen3 (required by skin libraries)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

## Find TCommon (TUM ICS common libraries)
find_package(TCommon REQUIRED)
add_definitions(${TCOMMON_DEFINITIONS})
include_directories(${TCOMMON_INCLUDE_DIRS})

## Find SkinCore (core skin libraries)
find_package(SkinCore REQUIRED)
include_directories(${SKINCORE_INCLUDE_DIRS})
add_definitions(${SKINCORE_DEFINITIONS})

## Find Qt5 - IMPORTANT: We need Widgets for QApplication!
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

## Generate messages
add_message_files(
  FILES
  FourCellForces.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    message_runtime 
    tum_ics_skin_msgs 
    tum_ics_skin_bridge
    tum_ics_skin_descr
)

## Specify additional include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

## Add executable
add_executable(force_publisher_node src/force_publisher_node.cpp)

## Add dependencies to ensure messages are generated first
add_dependencies(force_publisher_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${tum_ics_skin_msgs_EXPORTED_TARGETS}
)

## Link all required libraries - including all Qt5 modules
target_link_libraries(force_publisher_node
  ${catkin_LIBRARIES}
  ${SKINCORE_LIBRARIES}
  ${TCOMMON_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

## Enable Qt MOC (Meta-Object Compiler) for Qt signal/slot mechanism
set_target_properties(force_publisher_node PROPERTIES AUTOMOC TRUE)

catkin_install_python(PROGRAMS
  scripts/motor_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Set C++ standard
set_property(TARGET force_publisher_node PROPERTY CXX_STANDARD 14)