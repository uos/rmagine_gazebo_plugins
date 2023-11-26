cmake_minimum_required(VERSION 3.0.2)
project(rmagine_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

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

find_package(rmagine 2.2.1...
COMPONENTS
  core
OPTIONAL_COMPONENTS
  embree
  cuda
  optix
)


set(rmagine_gazebo_libs)

if(TARGET rmagine::embree)
  list(APPEND rmagine_gazebo_libs rmagine_embree_spherical_gzplugin)
  list(APPEND rmagine_gazebo_libs rmagine_embree_sensors_gzregister)
  list(APPEND rmagine_gazebo_libs rmagine_embree_map_gzplugin)
  list(APPEND rmagine_gazebo_libs rmagine_embree_ros_gzplugin)
endif(TARGET rmagine::embree)

if(TARGET rmagine::optix)
  list(APPEND rmagine_gazebo_libs rmagine_optix_spherical_gzplugin)
  list(APPEND rmagine_gazebo_libs rmagine_optix_sensors_gzregister)
  list(APPEND rmagine_gazebo_libs rmagine_optix_map_gzplugin)
  list(APPEND rmagine_gazebo_libs rmagine_optix_ros_gzplugin)
endif(TARGET rmagine::optix)

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
    ${rmagine_gazebo_libs}
  CATKIN_DEPENDS 
    ${GZ_RM_DEPS}
  DEPENDS
    rmagine
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
)

#### ROS-less Gazebo plugins

add_library(rmagine_gazebo_helper SHARED
  src/helper/helper_functions.cpp
  src/helper/SceneState.cpp    
  src/helper/SceneDiff.cpp
  src/helper/conversions.cpp
)

target_link_libraries(rmagine_gazebo_helper 
  ${GAZEBO_LIBRARIES}
  rmagine::core
)

if(TARGET rmagine::embree)

add_library(rmagine_embree_spherical_gzplugin SHARED
  src/rmagine_embree_spherical_gzplugin.cpp
)

target_link_libraries(rmagine_embree_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

add_library(rmagine_embree_sensors_gzregister SHARED
  src/rmagine_embree_spherical_gzregister.cpp
)

target_link_libraries(rmagine_embree_sensors_gzregister
  rmagine_embree_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

add_library(rmagine_embree_map_gzplugin SHARED
  src/rmagine_embree_map_gzplugin.cpp
  src/helper/embree_conversions.cpp
)

target_link_libraries(rmagine_embree_map_gzplugin 
  rmagine_gazebo_helper
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

# ROS Gazebo plugins
add_library(rmagine_embree_ros_gzplugin SHARED 
  src/rmagine_embree_ros_gzplugin.cpp)

target_link_libraries(rmagine_embree_ros_gzplugin 
  rmagine_embree_spherical_gzplugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

install(
  TARGETS 
    rmagine_embree_map_gzplugin
    rmagine_embree_spherical_gzplugin 
    rmagine_embree_sensors_gzregister
    rmagine_embree_ros_gzplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

endif(TARGET rmagine::embree)


### OPTIX
if(TARGET rmagine::optix)

#### ROS-less Gazebo plugins
add_library(rmagine_optix_spherical_gzplugin SHARED
  src/rmagine_optix_spherical_gzplugin.cpp
)

target_link_libraries(rmagine_optix_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

add_library(rmagine_optix_sensors_gzregister SHARED
  src/rmagine_optix_spherical_gzregister.cpp
)

target_link_libraries(rmagine_optix_sensors_gzregister
  rmagine_optix_spherical_gzplugin 
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

add_library(rmagine_optix_map_gzplugin SHARED
  src/rmagine_optix_map_gzplugin.cpp
  src/helper/optix_conversions.cpp
)

target_link_libraries(rmagine_optix_map_gzplugin 
  rmagine_optix_spherical_gzplugin
  rmagine_gazebo_helper
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

# ROS Gazebo plugins
add_library(rmagine_optix_ros_gzplugin SHARED 
  src/rmagine_optix_ros_gzplugin.cpp)

target_link_libraries(rmagine_optix_ros_gzplugin 
  rmagine_optix_spherical_gzplugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

install(
  TARGETS 
    rmagine_optix_map_gzplugin
    rmagine_optix_spherical_gzplugin 
    rmagine_optix_sensors_gzregister
    rmagine_optix_ros_gzplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

endif(TARGET rmagine::optix)