#include "rmagine_gazebo_plugins/gz/sensor_model_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace rmagine_gazebo_plugins
{

namespace
{

std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return std::tolower(c); });
  return s;
}

rmagine::Vector VectorFromYaml(const YAML::Node &node)
{
  rmagine::Vector v;
  v.x = node[0].as<float>();
  v.y = node[1].as<float>();
  v.z = node[2].as<float>();
  return v;
}

// YAML schema for O1Dn ("one shared origin, N directions"):
//
//   width: 8
//   height: 4
//   rays:
//     orig: [0, 0, 0]
//     dirs:
//       - [1, 0, 0]
//       - [0.99, 0.01, 0]
//       ...               # width * height entries, row-major
//                         # (vid * width + hid), matching getBufferId()
//
// and for OnDn ("N origins, N directions"), same top-level `rays` key but
// a per-ray `origs` list instead of a single shared `orig`:
//
//   width: 8
//   height: 4
//   rays:
//     origs:
//       - [0, 0, 0]
//       - [0, 0, 0]
//     dirs:
//       - [1, 0, 0]
//       - [0.99, 0.01, 0]
//       ...
YAML::Node LoadRaysFile(const std::string &rays_file)
{
  if(rays_file.empty())
  {
    std::cerr << "[SensorModelConfig] rays_file is required for O1Dn/OnDn model types." << std::endl;
    return YAML::Node();
  }

  try
  {
    return YAML::LoadFile(rays_file);
  }
  catch(const std::exception &e)
  {
    std::cerr << "[SensorModelConfig] Failed to load rays_file '" << rays_file
               << "': " << e.what() << std::endl;
    return YAML::Node();
  }
}

void LoadO1Dn(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::O1DnModel &model)
{
  model.width = 1;
  model.height = 1;
  model.orig = {0.0, 0.0, 0.0};
  model.dirs.resize(1);
  model.dirs[0] = {1.0, 0.0, 0.0};

  const std::string rays_file = _sdf->HasElement("rays_file")
    ? _sdf->Get<std::string>("rays_file") : std::string("");
  YAML::Node root = LoadRaysFile(rays_file);
  if(!root || !root["rays"] || !root["rays"]["dirs"] || !root["rays"]["dirs"].IsSequence())
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file
               << "' missing a 'rays.dirs' sequence -- falling back to a single forward-facing ray."
               << std::endl;
    return;
  }

  const uint32_t width = root["width"] ? root["width"].as<uint32_t>() : 1;
  const uint32_t height = root["height"] ? root["height"].as<uint32_t>() : 1;
  const YAML::Node rays = root["rays"];
  const YAML::Node dirs = rays["dirs"];
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  if(dirs.size() != expected)
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file << "' has " << dirs.size()
               << " dirs but width*height=" << expected << " -- using what's there." << std::endl;
  }

  model.width = width;
  model.height = height;
  model.orig = rays["orig"] ? VectorFromYaml(rays["orig"]) : rmagine::Vector{0.0, 0.0, 0.0};
  model.dirs.resize(dirs.size());
  for(size_t i = 0; i < dirs.size(); ++i)
  {
    model.dirs[i] = VectorFromYaml(dirs[i]);
  }
}

void LoadOnDn(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::OnDnModel &model)
{
  model.width = 1;
  model.height = 1;
  model.origs.resize(1);
  model.origs[0] = {0.0, 0.0, 0.0};
  model.dirs.resize(1);
  model.dirs[0] = {1.0, 0.0, 0.0};

  const std::string rays_file = _sdf->HasElement("rays_file")
    ? _sdf->Get<std::string>("rays_file") : std::string("");
  YAML::Node root = LoadRaysFile(rays_file);
  if(!root || !root["rays"] || !root["rays"]["origs"] || !root["rays"]["dirs"]
     || !root["rays"]["origs"].IsSequence() || !root["rays"]["dirs"].IsSequence())
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file
               << "' missing 'rays.origs'/'rays.dirs' sequences -- falling back to a single forward-facing ray."
               << std::endl;
    return;
  }

  const uint32_t width = root["width"] ? root["width"].as<uint32_t>() : 1;
  const uint32_t height = root["height"] ? root["height"].as<uint32_t>() : 1;
  const YAML::Node rays = root["rays"];
  const YAML::Node origs = rays["origs"];
  const YAML::Node dirs = rays["dirs"];
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  if(origs.size() != dirs.size())
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file << "' has " << origs.size()
               << " origs but " << dirs.size() << " dirs -- using the shorter count." << std::endl;
  }
  const size_t count = std::min(origs.size(), dirs.size());
  if(count != expected)
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file << "' has " << count
               << " rays but width*height=" << expected << " -- using what's there." << std::endl;
  }

  model.width = width;
  model.height = height;
  model.origs.resize(count);
  model.dirs.resize(count);
  for(size_t i = 0; i < count; ++i)
  {
    model.origs[i] = VectorFromYaml(origs[i]);
    model.dirs[i] = VectorFromYaml(dirs[i]);
  }
}

void LoadPinhole(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::PinholeModel &model,
  float range_min,
  float range_max)
{
  model.width = 100;
  model.height = 100;
  model.range.min = range_min;
  model.range.max = range_max;

  if(_sdf->HasElement("pinhole_width"))
  {
    model.width = _sdf->Get<unsigned int>("pinhole_width");
  }
  if(_sdf->HasElement("pinhole_height"))
  {
    model.height = _sdf->Get<unsigned int>("pinhole_height");
  }

  double hfov = 1.0472;  // ~60 degrees
  if(_sdf->HasElement("pinhole_hfov"))
  {
    hfov = _sdf->Get<double>("pinhole_hfov");
  }
  double vfov = hfov * static_cast<double>(model.height) / static_cast<double>(model.width);
  if(_sdf->HasElement("pinhole_vfov"))
  {
    vfov = _sdf->Get<double>("pinhole_vfov");
  }

  model.f[0] = static_cast<float>(model.width) / (2.0f * std::tan(static_cast<float>(hfov) / 2.0f));
  model.f[1] = static_cast<float>(model.height) / (2.0f * std::tan(static_cast<float>(vfov) / 2.0f));
  model.c[0] = static_cast<float>(model.width) / 2.0f;
  model.c[1] = static_cast<float>(model.height) / 2.0f;
}

// Reads a Classic-style <horizontal>/<vertical> scan axis element
// (<min_angle>, <increment>, <samples>) into a DiscreteInterval. Mirrors
// Gazebo Classic's fetch_sensor_model() in
// rmagine_embree_spherical_gzplugin.cpp, which read the identical tags
// nested the identical way -- restored here because the flat
// samples/min_angle/max_angle scheme this replaced had no way to express a
// vertical axis at all (phi was hardcoded to a single ring), silently
// reducing every "3D" spherical lidar to a 2D one.
void LoadScanAxis(
  const std::shared_ptr<sdf::Element> &axis_elem,
  rmagine::DiscreteInterval &axis)
{
  if(axis_elem->HasElement("min_angle"))
  {
    axis.min = axis_elem->Get<float>("min_angle");
  }
  if(axis_elem->HasElement("increment"))
  {
    axis.inc = axis_elem->Get<float>("increment");
  }
  if(axis_elem->HasElement("samples"))
  {
    axis.size = axis_elem->Get<unsigned int>("samples");
  }
}

void LoadSpherical(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::SphericalModel &model,
  float range_min,
  float range_max)
{
  // Defaults: a 400-sample horizontal ring, single vertical row -- matches
  // a common 2D rotating lidar and keeps every world that omits <scan>
  // entirely in a valid, usable state.
  model.phi.min = 0.0f;
  model.phi.inc = 1.0f;
  model.phi.size = 1;
  model.theta.min = -1.0472f;
  model.theta.inc = 0.01f;
  model.theta.size = 400;
  model.range.min = range_min;
  model.range.max = range_max;

  if(!_sdf || !_sdf->HasElement("scan"))
  {
    return;
  }

  auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
  auto scan_elem = sdf_mut->GetElement("scan");

  if(scan_elem->HasElement("horizontal"))
  {
    LoadScanAxis(scan_elem->GetElement("horizontal"), model.theta);
  }
  if(scan_elem->HasElement("vertical"))
  {
    LoadScanAxis(scan_elem->GetElement("vertical"), model.phi);
  }
}

}  // namespace

SensorModelConfig LoadSensorModelConfig(const std::shared_ptr<const sdf::Element> &_sdf)
{
  SensorModelConfig cfg;

  float range_min = 0.2f;
  float range_max = 100.0f;
  std::string model_type_str = "spherical";

  if(_sdf)
  {
    if(_sdf->HasElement("range_min"))
    {
      range_min = _sdf->Get<double>("range_min");
    }
    if(_sdf->HasElement("range_max"))
    {
      range_max = _sdf->Get<double>("range_max");
    }
    if(_sdf->HasElement("model_type"))
    {
      model_type_str = ToLower(_sdf->Get<std::string>("model_type"));
    }
  }

  if(model_type_str == "pinhole")
  {
    cfg.type = SensorModelType::Pinhole;
  } else if(model_type_str == "o1dn") {
    cfg.type = SensorModelType::O1Dn;
  } else if(model_type_str == "ondn") {
    cfg.type = SensorModelType::OnDn;
  } else {
    if(model_type_str != "spherical")
    {
      std::cerr << "[SensorModelConfig] Unknown model_type '" << model_type_str
                << "' -- falling back to 'spherical'." << std::endl;
    }
    cfg.type = SensorModelType::Spherical;
  }

  // Populate every model's defaults regardless of which is active -- keeps
  // each one always in a valid, usable state (harmless: only the active
  // model is ever handed to a simulator).
  LoadSpherical(_sdf, cfg.spherical, range_min, range_max);
  if(_sdf && cfg.type == SensorModelType::Pinhole)
  {
    LoadPinhole(_sdf, cfg.pinhole, range_min, range_max);
  }
  if(_sdf && cfg.type == SensorModelType::O1Dn)
  {
    LoadO1Dn(_sdf, cfg.o1dn);
    cfg.o1dn.range.min = range_min;
    cfg.o1dn.range.max = range_max;
  }
  if(_sdf && cfg.type == SensorModelType::OnDn)
  {
    LoadOnDn(_sdf, cfg.ondn);
    cfg.ondn.range.min = range_min;
    cfg.ondn.range.max = range_max;
  }

  return cfg;
}

}  // namespace rmagine_gazebo_plugins
