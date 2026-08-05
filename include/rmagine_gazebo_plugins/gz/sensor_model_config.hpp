#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_CONFIG_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_CONFIG_HPP

#include <memory>

#include <sdf/Element.hh>

#include <rmagine/types/sensor_models.h>

namespace rmagine_gazebo_plugins
{

// Which of rmagine's sensor models a sensor system instance is configured
// for. Fixed for the lifetime of one plugin instance -- chosen once from
// the SDF's `model_type` element at Configure() time, not switched at
// runtime.
enum class SensorModelType
{
  Spherical,
  Pinhole,
  O1Dn,
  OnDn
};

// Holds exactly one populated model, matching `type`. Kept as a plain
// struct with one member per model type (rather than a std::variant) to
// stay consistent with the rest of this codebase's style -- simple,
// explicit, no metaprogramming beyond the templated publish helpers in
// sensor_model_publish.hpp.
struct SensorModelConfig
{
  SensorModelType type{SensorModelType::Spherical};
  rmagine::SphericalModel spherical;
  rmagine::PinholeModel pinhole;
  rmagine::O1DnModel o1dn;
  rmagine::OnDnModel ondn;
};

// Parses `model_type` (default "spherical") plus that type's own SDF
// elements. Unknown `model_type` values fall back to spherical (logged to
// stderr) rather than failing sensor construction outright.
//
// Spherical: `<scan><horizontal>`/`<vertical>`, each with `<min_angle>`,
// `<increment>`, `<samples>` (Classic-style nested scan axes -- `vertical`
// is optional and defaults to a single ring, i.e. a 2D scan), plus
// `range_min`/`range_max`.
//
// Pinhole: `pinhole_width`/`pinhole_height` (pixel resolution),
// `pinhole_hfov` (radians; `pinhole_vfov` optional, derived from the
// aspect ratio if omitted), `range_min`/`range_max`.
//
// O1Dn/OnDn: `rays_file`, a YAML file (see sensor_model_config.cpp for the
// exact per-model format); these two models are
// defined by an arbitrary per-pixel ray set, not a closed-form formula
// like Spherical/Pinhole, so there's no reasonable small set of SDF
// scalars to expose instead.
SensorModelConfig LoadSensorModelConfig(const std::shared_ptr<const sdf::Element> &_sdf);

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_CONFIG_HPP
