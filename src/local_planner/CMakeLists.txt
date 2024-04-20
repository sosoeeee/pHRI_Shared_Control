cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  rospy
  roscpp
  geometry_msgs
  visualization_msgs
  tf2
  teb_local_planner
  # dynamic_reconfigure
  # global_planner
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LocalPlanning.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/TebLocalPlannerReconfigure.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES local_planner
 CATKIN_DEPENDS
  message_runtime 
  rospy
  roscpp
  geometry_msgs
  visualization_msgs
  tf2
  # global_planner
  teb_local_planner
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
# message(STATUS "Include directories: ${CMAKE_CURRENT_SOURCE_DIR}/include")
# message(STATUS "Include directories: ${catkin_INCLUDE_DIRS}")

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/teb_local_planner.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Dynamic reconfigure: make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(local_planner_node src/main.cpp)

add_dependencies(local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(local_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS local_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )
