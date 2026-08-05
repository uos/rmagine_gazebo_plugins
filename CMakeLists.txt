cmake_minimum_required(VERSION 3.5.0)
project(rmagine_gazebo_plugins)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

option(RMAGINE_GAZEBO_PLUGINS_BUILD_TESTING "Build tests" ON)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

## find ament and dependencies
find_package(ament_cmake REQUIRED)

# Locate the ROS distro's standard Gazebo generation: Harmonic (gz-sim8,
# Jazzy+) or Fortress (ignition-gazebo6, Humble). Fortress's own packages
# already ship gz::-namespaced headers identical to Harmonic's (ignition::
# is kept only as a forwarding compatibility layer on Fortress, confirmed
# against the actual installed headers, not just docs), so the only thing
# that differs at the C++ level is the plugin-registration macro; see
# include/rmagine_gazebo_plugins/gz/gz_compat.hpp.
find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  set(RMAGINE_GZ_SIM gz-sim8)
  set(RMAGINE_GZ_PLUGIN gz-plugin2)
  set(RMAGINE_GZ_TRANSPORT gz-transport13)
  set(RMAGINE_GZ_COMMON gz-common5)
  set(RMAGINE_GZ_MATH gz-math7)
  set(RMAGINE_GZ_MSGS gz-msgs10)
  set(RMAGINE_GZ_RENDERING gz-rendering8)
  set(RMAGINE_GZ_IGNITION_ERA 0)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(RMAGINE_GZ_SIM ignition-gazebo6)
  set(RMAGINE_GZ_PLUGIN ignition-plugin1)
  set(RMAGINE_GZ_TRANSPORT ignition-transport11)
  set(RMAGINE_GZ_COMMON ignition-common4)
  set(RMAGINE_GZ_MATH ignition-math6)
  set(RMAGINE_GZ_MSGS ignition-msgs8)
  set(RMAGINE_GZ_RENDERING ignition-rendering6)
  set(RMAGINE_GZ_IGNITION_ERA 1)
endif()

find_package(${RMAGINE_GZ_PLUGIN} REQUIRED)
find_package(${RMAGINE_GZ_TRANSPORT} REQUIRED)
find_package(${RMAGINE_GZ_COMMON} REQUIRED)
# Mesh/SubMesh/MeshManager (gz-common-native mesh loader fallback) live in a
# separate component from the base target.
find_package(${RMAGINE_GZ_COMMON} REQUIRED COMPONENTS graphics)
find_package(${RMAGINE_GZ_MATH} REQUIRED)
find_package(${RMAGINE_GZ_MSGS} REQUIRED)
find_package(${RMAGINE_GZ_RENDERING} REQUIRED)

set(RMAGINE_GZ_CORE_LIBS
  ${RMAGINE_GZ_SIM}::${RMAGINE_GZ_SIM}
  ${RMAGINE_GZ_PLUGIN}::${RMAGINE_GZ_PLUGIN}
  ${RMAGINE_GZ_TRANSPORT}::${RMAGINE_GZ_TRANSPORT}
  ${RMAGINE_GZ_COMMON}::${RMAGINE_GZ_COMMON}
  ${RMAGINE_GZ_MATH}::${RMAGINE_GZ_MATH}
  ${RMAGINE_GZ_MSGS}::${RMAGINE_GZ_MSGS}
  ${RMAGINE_GZ_RENDERING}::${RMAGINE_GZ_RENDERING}
)

find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmagine 2.3
COMPONENTS
  core
OPTIONAL_COMPONENTS
  embree
  cuda
  optix
  vulkan
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


## include dirs
include_directories(
  include
)


################
 ##  EMBREE  ##
################
if(TARGET rmagine::embree)
  message(STATUS "rmagine::embree found, building embree Gazebo plugins")
  
  add_library(rmagine_embree_map_system SHARED
    src/gz/rmagine_embree_map_system.cpp
  )

  target_include_directories(rmagine_embree_map_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(rmagine_embree_map_system
    ${RMAGINE_GZ_CORE_LIBS}
    ${RMAGINE_GZ_COMMON}::graphics
    rmagine::core
  )
  target_compile_definitions(rmagine_embree_map_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  target_link_libraries(rmagine_embree_map_system rmagine::embree)

  install(TARGETS rmagine_embree_map_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  add_library(rmagine_embree_sensor_system SHARED
    src/gz/rmagine_embree_sensor_system.cpp
    src/gz/sensor_model_config.cpp
  )

  target_include_directories(rmagine_embree_sensor_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(rmagine_embree_sensor_system
    ${RMAGINE_GZ_CORE_LIBS}
    rmagine::core
    yaml-cpp
  )
  target_compile_definitions(rmagine_embree_sensor_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  target_link_libraries(rmagine_embree_sensor_system rmagine::embree)

  install(TARGETS rmagine_embree_sensor_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

else()
  message(WARNING "rmagine::embree not found -- rmagine_embree_map_system/rmagine_embree_sensor_system will not be built.")
endif()


################
 ##  OPTIX  ##
################
if(TARGET rmagine::optix AND TARGET rmagine::cuda)
  add_library(rmagine_optix_map_system SHARED
    src/gz/rmagine_optix_map_system.cpp
  )

  target_include_directories(rmagine_optix_map_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(rmagine_optix_map_system
    ${RMAGINE_GZ_CORE_LIBS}
    ${RMAGINE_GZ_COMMON}::graphics
    rmagine::core
    rmagine::optix
  )
  target_compile_definitions(rmagine_optix_map_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  install(TARGETS rmagine_optix_map_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  add_library(rmagine_optix_sensor_system SHARED
    src/gz/rmagine_optix_sensor_system.cpp
    src/gz/sensor_model_config.cpp
  )

  target_include_directories(rmagine_optix_sensor_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(rmagine_optix_sensor_system
    ${RMAGINE_GZ_CORE_LIBS}
    rmagine::core
    rmagine::optix
    yaml-cpp
  )
  target_compile_definitions(rmagine_optix_sensor_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  target_link_libraries(rmagine_optix_sensor_system rmagine::cuda)

  install(TARGETS rmagine_optix_sensor_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "rmagine::optix not found -- rmagine_optix_map_system/rmagine_optix_sensor_system will not be built.")
endif()

# Ground Truth Localization Node
add_library(gt_localization SHARED
 src/ros/gt_localization.cpp)

target_include_directories(gt_localization
 PRIVATE
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gt_localization 
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_node(gt_localization
  PLUGIN "rmagine_gazebo_plugins::GTLocalizationNode"
  EXECUTABLE gt_localization_node)

install(TARGETS gt_localization 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(RMAGINE_GAZEBO_PLUGINS_BUILD_TESTING)

  add_library(test_box_mover_system SHARED
    tests/gz/test_box_mover_system.cpp
  )

  target_include_directories(test_box_mover_system PRIVATE
    tests/include
  )

  target_link_libraries(test_box_mover_system ${RMAGINE_GZ_CORE_LIBS})
  target_compile_definitions(test_box_mover_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  install(TARGETS test_box_mover_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  add_library(test_spawn_delete_system SHARED
    tests/gz/test_spawn_delete_system.cpp
  )

  target_include_directories(test_spawn_delete_system PRIVATE
    tests/include
  )

  target_link_libraries(test_spawn_delete_system ${RMAGINE_GZ_CORE_LIBS})
  target_compile_definitions(test_spawn_delete_system PRIVATE
    RMAGINE_GZ_IGNITION_ERA=${RMAGINE_GZ_IGNITION_ERA})

  install(TARGETS test_spawn_delete_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  install(
    PROGRAMS
      tests/scripts/embree_fixture_harness.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME capture_embree_fixture
  )

  install(
    PROGRAMS
      tests/scripts/embree_fixture_harness.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME compare_embree_fixture
  )

  install(
    PROGRAMS
      tests/scripts/check_mesh_cache_fixture.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME check_mesh_cache_fixture
  )

  install(
    DIRECTORY worlds urdf launch config rviz
    DESTINATION share/${PROJECT_NAME}
  )

  # test fixture worlds/data live under tests/, but still land in the same
  # installed share/worlds and share/testdata dirs as the demo worlds above;
  # the fixture harness (tests/scripts/embree_fixture_harness.py) resolves
  # both by name from the installed share directory, not by source location.
  install(
    DIRECTORY tests/worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
  )

  install(
    DIRECTORY tests/testdata
    DESTINATION share/${PROJECT_NAME}
  )

  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

  install(
    DIRECTORY include/
    DESTINATION include
  )
  ament_export_include_directories(include)

  if(BUILD_TESTING)
    find_package(ament_cmake_test REQUIRED)

    # APPEND_LIBRARY_DIRS "/usr/local/lib" below is a harmless leftover from
    # an earlier libiomp5.so-via-manual-symlink workaround; the real fix is
    # the libomp-dev package.xml dependency (see rmagine's embree backend:
    # its downloaded Embree+TBB binary expects libiomp5.so on the loader
    # path at runtime), which installs it into the standard system lib dir.
    ament_add_test(embree_fixture_baseline
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture baseline && ros2 run rmagine_gazebo_plugins compare_embree_fixture baseline"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_dynamic
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture dynamic && ros2 run rmagine_gazebo_plugins compare_embree_fixture dynamic"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 180
    )

    ament_add_test(embree_fixture_pinhole
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture pinhole && ros2 run rmagine_gazebo_plugins compare_embree_fixture pinhole"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_o1dn
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture o1dn && ros2 run rmagine_gazebo_plugins compare_embree_fixture o1dn"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_ondn
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture ondn && ros2 run rmagine_gazebo_plugins compare_embree_fixture ondn"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_plane
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture plane && ros2 run rmagine_gazebo_plugins compare_embree_fixture plane"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_heightmap
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture heightmap && ros2 run rmagine_gazebo_plugins compare_embree_fixture heightmap"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_ignore_link
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture ignore_link && ros2 run rmagine_gazebo_plugins compare_embree_fixture ignore_link"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_vertical
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture vertical && ros2 run rmagine_gazebo_plugins compare_embree_fixture vertical"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    ament_add_test(embree_fixture_zombie
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture zombie && ros2 run rmagine_gazebo_plugins compare_embree_fixture zombie"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    if(TARGET rmagine::optix)
      ament_add_test(optix_fixture_baseline
        COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_baseline && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_baseline"
        APPEND_LIBRARY_DIRS "/usr/local/lib"
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        TIMEOUT 120
      )

      ament_add_test(optix_fixture_dynamic
        COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_dynamic && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_dynamic"
        APPEND_LIBRARY_DIRS "/usr/local/lib"
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        TIMEOUT 180
      )

      ament_add_test(optix_fixture_noise
        COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_noise && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_noise"
        APPEND_LIBRARY_DIRS "/usr/local/lib"
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        TIMEOUT 120
      )

      ament_add_test(optix_fixture_multi
        COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_multi && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_multi"
        APPEND_LIBRARY_DIRS "/usr/local/lib"
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        TIMEOUT 120
      )
    endif()

    ament_add_test(embree_fixture_mesh_cache
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins check_mesh_cache_fixture"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 60
    )
  endif()

endif()

ament_package()
