cmake_minimum_required(VERSION 3.5.0)
project(rmagine_gazebo_plugins)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

option(RMAGINE_GZSIM_PORT "Build gz-sim ported plugins instead of Classic plugins" ON)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

## find ament and dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-common5 REQUIRED)
# Mesh/SubMesh/MeshManager (used for the gz-common-native mesh loader
# fallback, see BuildEmbreeMeshFromGzCommon/BuildOptixMeshFromGzCommon)
# live in a separate component from the base gz-common5 target -- gz::common::Image
# (used for heightmaps) is in the base component and links fine without
# this, which is why this was easy to miss initially.
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmagine 2.3
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
)

if(RMAGINE_GZSIM_PORT)
  add_library(rmagine_embree_map_system SHARED
    src/gz/rmagine_embree_map_system.cpp
  )

  target_include_directories(rmagine_embree_map_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(rmagine_embree_map_system
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-common5::gz-common5
    gz-common5::graphics
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-rendering8::gz-rendering8
    rmagine::core
  )

  if(TARGET rmagine::embree)
    target_link_libraries(rmagine_embree_map_system rmagine::embree)
  endif()

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
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-rendering8::gz-rendering8
    rmagine::core
    yaml-cpp
  )

  ament_target_dependencies(rmagine_embree_sensor_system
    geometry_msgs
    rclcpp
    sensor_msgs
    tf2_ros
  )

  if(TARGET rmagine::embree)
    target_link_libraries(rmagine_embree_sensor_system rmagine::embree)
  endif()

  install(TARGETS rmagine_embree_sensor_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  if(TARGET rmagine::optix)
    add_library(rmagine_optix_map_system SHARED
      src/gz/rmagine_optix_map_system.cpp
    )

    target_include_directories(rmagine_optix_map_system PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(rmagine_optix_map_system
      gz-sim8::gz-sim8
      gz-plugin2::gz-plugin2
      gz-transport13::gz-transport13
      gz-common5::gz-common5
      gz-common5::graphics
      gz-math7::gz-math7
      gz-msgs10::gz-msgs10
      gz-rendering8::gz-rendering8
      rmagine::core
      rmagine::optix
    )

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
      gz-sim8::gz-sim8
      gz-plugin2::gz-plugin2
      gz-transport13::gz-transport13
      gz-common5::gz-common5
      gz-math7::gz-math7
      gz-msgs10::gz-msgs10
      gz-rendering8::gz-rendering8
      rmagine::core
      rmagine::optix
      yaml-cpp
    )

    if(TARGET rmagine::cuda)
      # GaussianNoiseCuda/UniformDustNoiseCuda/RelGaussianNoiseCuda live in
      # rmagine_cuda (a separate component from rmagine_optix) -- needed to
      # port Classic's OptiX noise support, see LoadParams()'s <noise>
      # parsing in rmagine_optix_sensor_system.cpp.
      target_link_libraries(rmagine_optix_sensor_system rmagine::cuda)
    else()
      message(WARNING "rmagine::cuda not found -- rmagine_optix_sensor_system will build without noise model support.")
    endif()

    ament_target_dependencies(rmagine_optix_sensor_system
      geometry_msgs
      rclcpp
      sensor_msgs
      tf2_ros
    )

    install(TARGETS rmagine_optix_sensor_system
      LIBRARY DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "rmagine::optix not found -- rmagine_optix_map_system/rmagine_optix_sensor_system will not be built.")
  endif()

  add_library(test_box_mover_system SHARED
    src/gz/test_box_mover_system.cpp
  )

  target_include_directories(test_box_mover_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(test_box_mover_system
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-rendering8::gz-rendering8
  )

  install(TARGETS test_box_mover_system
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
else()
  #### ROS-less Gazebo plugins (Classic)
  add_library(rmagine_gazebo_helper SHARED
    src/helper/helper_functions.cpp
    src/helper/SceneState.cpp    
    src/helper/SceneDiff.cpp
    src/helper/conversions.cpp
  )

  target_link_libraries(rmagine_gazebo_helper 
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-rendering8::gz-rendering8
    rmagine::core
  )

  install(TARGETS rmagine_gazebo_helper
      LIBRARY DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(NOT RMAGINE_GZSIM_PORT AND TARGET rmagine::embree)

add_library(rmagine_embree_spherical_gzplugin SHARED
  src/rmagine_embree_spherical_gzplugin.cpp
)

target_link_libraries(rmagine_embree_spherical_gzplugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::embree
)

add_library(rmagine_embree_sensors_gzregister SHARED
  src/rmagine_embree_spherical_gzregister.cpp
)

target_link_libraries(rmagine_embree_sensors_gzregister
  rmagine_embree_spherical_gzplugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::embree
)

add_library(rmagine_embree_map_gzplugin SHARED
  src/rmagine_embree_map_gzplugin.cpp
  src/helper/embree_conversions.cpp
)

target_link_libraries(rmagine_embree_map_gzplugin 
  rmagine_gazebo_helper
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::embree
  # with the current implementation the map notifies
  # the sensors when the map was changed. Therefore,
  # it depends on them:
  rmagine_embree_spherical_gzplugin
)

# ROS Gazebo plugins
add_library(rmagine_embree_ros_gzplugin SHARED 
  src/rmagine_embree_ros_gzplugin.cpp)

target_link_libraries(rmagine_embree_ros_gzplugin 
  rmagine_embree_spherical_gzplugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::embree
)

# Link ROS2
ament_target_dependencies(rmagine_embree_ros_gzplugin 
  rclcpp 
  sensor_msgs)

install(
  TARGETS 
    rmagine_embree_map_gzplugin
    rmagine_embree_spherical_gzplugin 
    rmagine_embree_sensors_gzregister
    rmagine_embree_ros_gzplugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

endif()


### OPTIX
if(NOT RMAGINE_GZSIM_PORT AND TARGET rmagine::optix)

#### ROS-less Gazebo plugins
add_library(rmagine_optix_spherical_gzplugin SHARED
  src/rmagine_optix_spherical_gzplugin.cpp
)

target_link_libraries(rmagine_optix_spherical_gzplugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::optix
)

add_library(rmagine_optix_sensors_gzregister SHARED
  src/rmagine_optix_spherical_gzregister.cpp
)

target_link_libraries(rmagine_optix_sensors_gzregister
  rmagine_optix_spherical_gzplugin 
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::optix
)

add_library(rmagine_optix_map_gzplugin SHARED
  src/rmagine_optix_map_gzplugin.cpp
  src/helper/optix_conversions.cpp
)

target_link_libraries(rmagine_optix_map_gzplugin 
  rmagine_optix_spherical_gzplugin
  rmagine_gazebo_helper
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
  rmagine::optix
)

# ROS Gazebo plugins
add_library(rmagine_optix_ros_gzplugin SHARED 
  src/rmagine_optix_ros_gzplugin.cpp)

target_link_libraries(rmagine_optix_ros_gzplugin 
  rmagine_optix_spherical_gzplugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-rendering8::gz-rendering8
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
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

endif()

# Install launch files
install(
  PROGRAMS
    scripts/embree_fixture_harness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_embree_fixture
)

install(
  PROGRAMS
    scripts/embree_fixture_harness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compare_embree_fixture
)

install(
  PROGRAMS
    scripts/check_mesh_cache_fixture.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_mesh_cache_fixture
)

install(
  DIRECTORY launch worlds urdf testdata
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Header-only helpers (e.g. gz/map_registry.hpp) are shared with other
# packages so they can delegate map/sensor lifecycle instead of duplicating
# it. See radarays_gazebo_plugins/MIGRATION.md.
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

if(BUILD_TESTING AND RMAGINE_GZSIM_PORT)
  # Wires the existing capture/compare fixture harness into `colcon test`
  # instead of only ever running by hand or via ci_build_and_test.sh --
  # see MIGRATION_HANDOFF.md, "Wired fixtures into colcon test". Each test
  # captures a fresh run of the world and compares it against the
  # committed fixture in testdata/embree_harmonic/ in one shot.
  #
  # /usr/local/lib is where the libiomp5.so workaround symlink lives (see
  # "Known Environment Issues" in MIGRATION_HANDOFF.md) -- APPEND_LIBRARY_DIRS
  # puts it on LD_LIBRARY_PATH for the test process, matching what
  # ci_build_and_test.sh does at the shell level.
  find_package(ament_cmake_test REQUIRED)

  # Matches how these scripts are invoked everywhere else in this
  # workspace (ci_build_and_test.sh, manual testing this session) --
  # `ros2 run`, not a bare PATH lookup; the installed script lives under
  # lib/${PROJECT_NAME}, which isn't on PATH by itself.
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

  # Regression coverage for the Pinhole/O1Dn/OnDn model_type support (see
  # README.md, "Non-spherical sensor models (Pinhole/O1Dn/OnDn)") -- same
  # capture/compare harness and target_box scene as the Spherical fixtures
  # above, just a different model_type per world. o1dn/ondn's rays_file
  # SDF element is resolved from a placeholder to the real installed
  # testdata path at launch time (see resolved_world_path() in
  # embree_fixture_harness.py).
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

  # Regression coverage for the Tier1/2/3 Classic-parity gaps closed this
  # migration that had no automated test before today (plane geometry,
  # heightmap geometry, per-link ignore) -- previously only verified by
  # ad-hoc scratchpad SDF worlds that don't persist between sessions. See
  # README.md for the individual feature writeups.
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

  # Falsifiable check on the ignore_link feature itself, not just
  # crash-freedom: if ignore_link regresses, the ignored (closer) box
  # would occlude the real target and center_value would read ~1.8
  # instead of ~2.5 -- see gz_embree_ignore_link.sdf and
  # ignore_link_fixture.json's reference_summary for the exact numbers.
  ament_add_test(embree_fixture_ignore_link
    COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture ignore_link && ros2 run rmagine_gazebo_plugins compare_embree_fixture ignore_link"
    APPEND_LIBRARY_DIRS "/usr/local/lib"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )

  # OptiX/GPU regression coverage -- gz_optix_baseline.sdf/gz_optix_dynamic.sdf
  # already existed on disk before this session but were never wired into
  # colcon test at all, meaning this package's entire OptiX/GPU path
  # (rmagine_optix_map_system, rmagine_optix_sensor_system) had zero
  # automated coverage. Same scenes as embree_fixture_baseline/dynamic
  # (only the plugin filenames differ), so the same expected fixture
  # values apply -- see README.md for the real numbers this was verified
  # against. Gated on rmagine::optix being available, same as the plugin
  # targets themselves above.
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

    # Falsifiable check on the noise models themselves (README.md, "OptiX
    # noise models"), not just crash-freedom: optix_baseline's own
    # center_value_stddev is ~0 (deterministic), so if this world's isn't
    # clearly above that floor, the <noise> element isn't being applied.
    ament_add_test(optix_fixture_noise
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_noise && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_noise"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )

    # Falsifiable check on GPU multi-topic fan-out itself: the extra
    # scan2/points2 topics must ALSO receive messages, not just the
    # primary topic_scan/topic_points.
    ament_add_test(optix_fixture_multi
      COMMAND bash -c "ros2 run rmagine_gazebo_plugins capture_embree_fixture optix_multi && ros2 run rmagine_gazebo_plugins compare_embree_fixture optix_multi"
      APPEND_LIBRARY_DIRS "/usr/local/lib"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      TIMEOUT 120
    )
  endif()

  # Mesh-by-URI caching (Tier3 #7) -- the one Classic-parity feature that
  # never fit the capture/compare-JSON pattern the fixtures above use,
  # since it's about an internal load COUNT, not published topic values.
  # See check_mesh_cache_fixture.py and gz_embree_mesh_cache.sdf.
  ament_add_test(embree_fixture_mesh_cache
    COMMAND bash -c "ros2 run rmagine_gazebo_plugins check_mesh_cache_fixture"
    APPEND_LIBRARY_DIRS "/usr/local/lib"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
endif()

ament_package()
