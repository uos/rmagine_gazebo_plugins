cmake_minimum_required(VERSION 3.5.0)
project(rmagine_gazebo_plugins)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

## find ament and dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmagine 2.2.1...
COMPONENTS
  core
OPTIONAL_COMPONENTS
  embree
  cuda
  optix
)

## include dirs
include_directories(
  include
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
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

# Link ROS2
ament_target_dependencies(rmagine_embree_ros_gzplugin rclcpp sensor_msgs)

install(
  TARGETS 
    rmagine_embree_map_gzplugin
    rmagine_embree_spherical_gzplugin 
    rmagine_embree_sensors_gzregister
    rmagine_embree_ros_gzplugin
  LIBRARY DESTINATION lib
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
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)
# Link ROS2
ament_target_dependencies(rmagine_optix_ros_gzplugin rclcpp sensor_msgs)

install(
  TARGETS 
    rmagine_optix_map_gzplugin
    rmagine_optix_spherical_gzplugin 
    rmagine_optix_sensors_gzregister
    rmagine_optix_ros_gzplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

endif(TARGET rmagine::optix)

# Install launch files
install(
  DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
