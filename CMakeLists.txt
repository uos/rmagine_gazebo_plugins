cmake_minimum_required(VERSION 3.0.2)
project(rmagine_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(GZ_RM_DEPS
  gazebo_ros
  roscpp
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${GZ_RM_DEPS}
)

find_package(gazebo REQUIRED)

find_package(rmagine REQUIRED)

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
  INCLUDE_DIRS 
    include
  LIBRARIES 
    gazebo_rmagine_map_plugin
  CATKIN_DEPENDS 
    ${GZ_RM_DEPS}
)

# add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${rmagine_INCLUDE_DIRS}
)

#### ROS-less Gazebo plugins

add_library(rmagine_embree_spherical_gzplugin SHARED
  src/rmagine_embree_spherical_gzplugin.cpp
)

target_link_libraries(rmagine_embree_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${rmagine_LIBRARIES}
)


add_library(rmagine_embree_spherical_gzregister SHARED
  src/rmagine_embree_spherical_gzregister.cpp
)

target_link_libraries(rmagine_embree_spherical_gzregister
  rmagine_embree_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${rmagine_LIBRARIES}
)

add_library(rmagine_embree_map_gzplugin SHARED
  src/rmagine_embree_map_gzplugin.cpp
)

target_link_libraries(rmagine_embree_map_gzplugin 
  rmagine_embree_spherical_gzplugin
  ${GAZEBO_LIBRARIES}
  ${rmagine_LIBRARIES}
)

# ROS Gazebo plugins
add_library(rmagine_embree_ros_gzplugin SHARED 
  src/rmagine_embree_ros_gzplugin.cpp)
target_link_libraries(rmagine_embree_ros_gzplugin 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${rmagine_LIBRARIES}
)

install(
  TARGETS 
    rmagine_embree_map_gzplugin
    rmagine_embree_spherical_gzplugin 
    rmagine_embree_spherical_gzregister
    rmagine_embree_ros_gzplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)