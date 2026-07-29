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

// Shared YAML schema for both O1Dn ("one origin, N directions") and OnDn
// ("N origins, N directions"):
//
//   width: 8
//   height: 4
//   rays:
//     - origin: [0, 0, 0]   # O1Dn: only the first ray's origin is used
//                           # (all rays share one origin); OnDn: read per ray
//       dir: [1, 0, 0]
//     - origin: [0, 0, 0]
//       dir: [0.99, 0.01, 0]
//     ...                   # width * height entries, row-major
//                           # (vid * width + hid), matching getBufferId()
//
// A single shared file format (rather than two subtly different ones) so
// the same file can be repurposed between O1Dn and OnDn by just switching
// `model_type` -- O1Dn simply ignores every origin but the first.
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
  if(!root || !root["rays"] || !root["rays"].IsSequence())
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file
               << "' missing a 'rays' sequence -- falling back to a single forward-facing ray."
               << std::endl;
    return;
  }

  const uint32_t width = root["width"] ? root["width"].as<uint32_t>() : 1;
  const uint32_t height = root["height"] ? root["height"].as<uint32_t>() : 1;
  const YAML::Node rays = root["rays"];
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  if(rays.size() != expected)
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file << "' has " << rays.size()
               << " rays but width*height=" << expected << " -- using what's there." << std::endl;
  }

  model.width = width;
  model.height = height;
  model.dirs.resize(rays.size());
  model.orig = rays[0]["origin"] ? VectorFromYaml(rays[0]["origin"]) : rmagine::Vector{0.0, 0.0, 0.0};
  for(size_t i = 0; i < rays.size(); ++i)
  {
    model.dirs[i] = VectorFromYaml(rays[i]["dir"]);
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
  if(!root || !root["rays"] || !root["rays"].IsSequence())
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file
               << "' missing a 'rays' sequence -- falling back to a single forward-facing ray."
               << std::endl;
    return;
  }

  const uint32_t width = root["width"] ? root["width"].as<uint32_t>() : 1;
  const uint32_t height = root["height"] ? root["height"].as<uint32_t>() : 1;
  const YAML::Node rays = root["rays"];
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  if(rays.size() != expected)
  {
    std::cerr << "[SensorModelConfig] rays_file '" << rays_file << "' has " << rays.size()
               << " rays but width*height=" << expected << " -- using what's there." << std::endl;
  }

  model.width = width;
  model.height = height;
  model.origs.resize(rays.size());
  model.dirs.resize(rays.size());
  for(size_t i = 0; i < rays.size(); ++i)
  {
    model.origs[i] = rays[i]["origin"] ? VectorFromYaml(rays[i]["origin"]) : rmagine::Vector{0.0, 0.0, 0.0};
    model.dirs[i] = VectorFromYaml(rays[i]["dir"]);
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

void LoadSpherical(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::SphericalModel &model,
  float range_min,
  float range_max)
{
  // Defaults match the pre-existing (single-model) sensor system behavior.
  model.phi.min = 0.0;
  model.phi.inc = 1.0;
  model.phi.size = 1;
  model.theta.min = -1.0472;
  model.theta.inc = 0.01;
  model.theta.size = 400;
  model.range.min = range_min;
  model.range.max = range_max;

  if(!_sdf)
  {
    return;
  }

  if(_sdf->HasElement("samples"))
  {
    model.theta.size = _sdf->Get<unsigned int>("samples");
  }
  if(_sdf->HasElement("min_angle"))
  {
    model.theta.min = _sdf->Get<double>("min_angle");
  }
  if(_sdf->HasElement("max_angle"))
  {
    const double max_angle = _sdf->Get<double>("max_angle");
    if(model.theta.size > 1)
    {
      model.theta.inc = static_cast<float>(
        (max_angle - model.theta.min) / static_cast<double>(model.theta.size - 1));
    }
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
