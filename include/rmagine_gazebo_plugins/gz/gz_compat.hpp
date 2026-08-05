#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_GZ_COMPAT_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_GZ_COMPAT_HPP

// Bridges the one real API difference between Gazebo Harmonic (gz-sim8,
// Jazzy's default) and Gazebo Fortress (ignition-gazebo6, Humble's default):
// the plugin-registration macro. Everything else this package touches,
// the gz::sim/gz::common/gz::math/gz::msgs/gz::transport namespaces and
// <gz/...> include paths, is identical on both, since Fortress's own
// packages ship gz::-namespaced headers with ignition:: kept only as a
// forwarding compatibility layer. RMAGINE_GZ_IGNITION_ERA is set by
// CMakeLists.txt based on which generation of packages was found.
#include <gz/plugin/Register.hh>

#if RMAGINE_GZ_IGNITION_ERA
  #define RMAGINE_GZ_ADD_PLUGIN(...) IGNITION_ADD_PLUGIN(__VA_ARGS__)
  #define RMAGINE_GZ_ADD_PLUGIN_ALIAS(...) IGNITION_ADD_PLUGIN_ALIAS(__VA_ARGS__)
#else
  #define RMAGINE_GZ_ADD_PLUGIN(...) GZ_ADD_PLUGIN(__VA_ARGS__)
  #define RMAGINE_GZ_ADD_PLUGIN_ALIAS(...) GZ_ADD_PLUGIN_ALIAS(__VA_ARGS__)
#endif

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_GZ_COMPAT_HPP
