#include "rmagine_gazebo_plugins/gz/sensor_model_config.hpp"

#include <yaml-cpp/yaml.h>

#include <gz/math/Vector3.hh>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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

// Reads a "x y z" space-separated vector from a single SDF element's own
// text content -- gz-sim's own established convention for vector-valued
// SDF elements (e.g. <axis><xyz>0 0 1</xyz></axis>).
rmagine::Vector VectorFromSdf(const std::shared_ptr<sdf::Element> &elem)
{
  const gz::math::Vector3d v = elem->Get<gz::math::Vector3d>();
  return rmagine::Vector{
    static_cast<float>(v.X()), static_cast<float>(v.Y()), static_cast<float>(v.Z())};
}

// Inline alternative to <rays_file>, for small hand-authored ray sets that
// don't warrant a separate YAML file. Mirrors the YAML schema's `rays: {
// orig|origs, dirs }` shape as nested SDF elements, one repeated <dir>/
// <orig> per ray instead of a YAML list:
//
//   <rays>
//     <orig>0 0 0</orig>        <!-- O1Dn only: one shared origin -->
//     <origs>                   <!-- OnDn only: one <orig> per ray -->
//       <orig>0 0 0</orig>
//       <orig>0 0 0</orig>
//     </origs>
//     <dirs>
//       <dir>1 0 0</dir>
//       <dir>0.99 0.01 0</dir>
//     </dirs>
//   </rays>
std::vector<rmagine::Vector> LoadInlineVectorList(
  const std::shared_ptr<sdf::Element> &parent_elem,
  const std::string &list_tag,
  const std::string &item_tag)
{
  std::vector<rmagine::Vector> vecs;
  if(!parent_elem->HasElement(list_tag))
  {
    return vecs;
  }
  auto list_elem = parent_elem->GetElement(list_tag);
  if(!list_elem->HasElement(item_tag))
  {
    return vecs;
  }
  for(auto item_elem = list_elem->GetElement(item_tag); item_elem;
      item_elem = item_elem->GetNextElement(item_tag))
  {
    vecs.push_back(VectorFromSdf(item_elem));
  }
  return vecs;
}

// Reads <range><min>/<max></range> from the given wrapper element (mirrors
// gz-sim's own <lidar><range> convention), defaulting to the given values
// if <range>, or the wrapper element itself, is absent.
void LoadRange(
  const std::shared_ptr<sdf::Element> &wrapper_elem,
  rmagine::Interval &range,
  float default_min,
  float default_max)
{
  range.min = default_min;
  range.max = default_max;
  if(!wrapper_elem || !wrapper_elem->HasElement("range"))
  {
    return;
  }
  auto range_elem = wrapper_elem->GetElement("range");
  if(range_elem->HasElement("min"))
  {
    range.min = range_elem->Get<float>("min");
  }
  if(range_elem->HasElement("max"))
  {
    range.max = range_elem->Get<float>("max");
  }
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
// Empty rays_file (no <rays_file> given) is not an error here -- the caller
// falls back to inline <rays> under <scan> in that case, see LoadO1Dn/
// LoadOnDn.
YAML::Node LoadRaysFile(const std::string &rays_file)
{
  if(rays_file.empty())
  {
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
  model.range.min = 0.2f;
  model.range.max = 100.0f;

  if(!_sdf || !_sdf->HasElement("o1dn"))
  {
    return;
  }
  auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
  auto o1dn_elem = sdf_mut->GetElement("o1dn");
  LoadRange(o1dn_elem, model.range, 0.2f, 100.0f);

  if(!o1dn_elem->HasElement("scan"))
  {
    return;
  }
  auto scan_elem = o1dn_elem->GetElement("scan");
  const uint32_t width = scan_elem->HasElement("width") ? scan_elem->Get<uint32_t>("width") : 1;
  const uint32_t height = scan_elem->HasElement("height") ? scan_elem->Get<uint32_t>("height") : 1;
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  const std::string rays_file = scan_elem->HasElement("rays_file")
    ? scan_elem->Get<std::string>("rays_file") : std::string("");
  if(!rays_file.empty())
  {
    YAML::Node root = LoadRaysFile(rays_file);
    if(!root || !root["rays"] || !root["rays"]["dirs"] || !root["rays"]["dirs"].IsSequence())
    {
      std::cerr << "[SensorModelConfig] rays_file '" << rays_file
                 << "' missing a 'rays.dirs' sequence -- falling back to a single forward-facing ray."
                 << std::endl;
      return;
    }

    const YAML::Node rays = root["rays"];
    const YAML::Node dirs = rays["dirs"];
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
    return;
  }

  // No <rays_file>: fall back to inline <rays> directly under <scan>.
  if(scan_elem->HasElement("rays"))
  {
    auto rays_elem = scan_elem->GetElement("rays");
    std::vector<rmagine::Vector> dirs = LoadInlineVectorList(rays_elem, "dirs", "dir");
    if(!dirs.empty())
    {
      if(dirs.size() != expected)
      {
        std::cerr << "[SensorModelConfig] inline rays has " << dirs.size()
                   << " dirs but width*height=" << expected << " -- using what's there." << std::endl;
      }
      model.width = width;
      model.height = height;
      model.orig = rays_elem->HasElement("orig")
        ? VectorFromSdf(rays_elem->GetElement("orig")) : rmagine::Vector{0.0, 0.0, 0.0};
      model.dirs.resize(dirs.size());
      for(size_t i = 0; i < dirs.size(); ++i)
      {
        model.dirs[i] = dirs[i];
      }
      return;
    }
  }

  std::cerr << "[SensorModelConfig] o1dn sensor has neither a rays_file nor inline rays under "
               "<scan> -- falling back to a single forward-facing ray." << std::endl;
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
  model.range.min = 0.2f;
  model.range.max = 100.0f;

  if(!_sdf || !_sdf->HasElement("ondn"))
  {
    return;
  }
  auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
  auto ondn_elem = sdf_mut->GetElement("ondn");
  LoadRange(ondn_elem, model.range, 0.2f, 100.0f);

  if(!ondn_elem->HasElement("scan"))
  {
    return;
  }
  auto scan_elem = ondn_elem->GetElement("scan");
  const uint32_t width = scan_elem->HasElement("width") ? scan_elem->Get<uint32_t>("width") : 1;
  const uint32_t height = scan_elem->HasElement("height") ? scan_elem->Get<uint32_t>("height") : 1;
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);

  const std::string rays_file = scan_elem->HasElement("rays_file")
    ? scan_elem->Get<std::string>("rays_file") : std::string("");
  if(!rays_file.empty())
  {
    YAML::Node root = LoadRaysFile(rays_file);
    if(!root || !root["rays"] || !root["rays"]["origs"] || !root["rays"]["dirs"]
       || !root["rays"]["origs"].IsSequence() || !root["rays"]["dirs"].IsSequence())
    {
      std::cerr << "[SensorModelConfig] rays_file '" << rays_file
                 << "' missing 'rays.origs'/'rays.dirs' sequences -- falling back to a single forward-facing ray."
                 << std::endl;
      return;
    }

    const YAML::Node rays = root["rays"];
    const YAML::Node origs = rays["origs"];
    const YAML::Node dirs = rays["dirs"];
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
    return;
  }

  // No <rays_file>: fall back to inline <rays> directly under <scan>.
  if(scan_elem->HasElement("rays"))
  {
    auto rays_elem = scan_elem->GetElement("rays");
    std::vector<rmagine::Vector> origs = LoadInlineVectorList(rays_elem, "origs", "orig");
    std::vector<rmagine::Vector> dirs = LoadInlineVectorList(rays_elem, "dirs", "dir");
    if(!origs.empty() && !dirs.empty())
    {
      if(origs.size() != dirs.size())
      {
        std::cerr << "[SensorModelConfig] inline rays has " << origs.size()
                   << " origs but " << dirs.size() << " dirs -- using the shorter count." << std::endl;
      }
      const size_t count = std::min(origs.size(), dirs.size());
      if(count != expected)
      {
        std::cerr << "[SensorModelConfig] inline rays has " << count
                   << " rays but width*height=" << expected << " -- using what's there." << std::endl;
      }
      model.width = width;
      model.height = height;
      model.origs.resize(count);
      model.dirs.resize(count);
      for(size_t i = 0; i < count; ++i)
      {
        model.origs[i] = origs[i];
        model.dirs[i] = dirs[i];
      }
      return;
    }
  }

  std::cerr << "[SensorModelConfig] ondn sensor has neither a rays_file nor inline rays under "
               "<scan> -- falling back to a single forward-facing ray." << std::endl;
}

void LoadPinhole(
  const std::shared_ptr<const sdf::Element> &_sdf,
  rmagine::PinholeModel &model)
{
  model.width = 100;
  model.height = 100;
  model.range.min = 0.2f;
  model.range.max = 100.0f;

  double hfov = 1.0472;  // ~60 degrees
  double vfov = -1.0;    // < 0: derive from hfov and the aspect ratio below

  if(!_sdf || !_sdf->HasElement("pinhole"))
  {
    vfov = hfov * static_cast<double>(model.height) / static_cast<double>(model.width);
    model.f[0] = static_cast<float>(model.width) / (2.0f * std::tan(static_cast<float>(hfov) / 2.0f));
    model.f[1] = static_cast<float>(model.height) / (2.0f * std::tan(static_cast<float>(vfov) / 2.0f));
    model.c[0] = static_cast<float>(model.width) / 2.0f;
    model.c[1] = static_cast<float>(model.height) / 2.0f;
    return;
  }
  auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
  auto pinhole_elem = sdf_mut->GetElement("pinhole");
  LoadRange(pinhole_elem, model.range, 0.2f, 100.0f);

  if(pinhole_elem->HasElement("scan"))
  {
    auto scan_elem = pinhole_elem->GetElement("scan");
    if(scan_elem->HasElement("width"))
    {
      model.width = scan_elem->Get<unsigned int>("width");
    }
    if(scan_elem->HasElement("height"))
    {
      model.height = scan_elem->Get<unsigned int>("height");
    }
    if(scan_elem->HasElement("hfov"))
    {
      hfov = scan_elem->Get<double>("hfov");
    }
    if(scan_elem->HasElement("vfov"))
    {
      vfov = scan_elem->Get<double>("vfov");
    }
  }

  if(vfov < 0.0)
  {
    vfov = hfov * static_cast<double>(model.height) / static_cast<double>(model.width);
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
  rmagine::SphericalModel &model)
{
  // Defaults: a 400-sample horizontal ring, single vertical row. Matches a
  // common 2D rotating lidar and keeps every world that omits <lidar>
  // entirely in a valid, usable state.
  model.phi.min = 0.0f;
  model.phi.inc = 1.0f;
  model.phi.size = 1;
  model.theta.min = -1.0472f;
  model.theta.inc = 0.01f;
  model.theta.size = 400;
  model.range.min = 0.2f;
  model.range.max = 100.0f;

  if(!_sdf || !_sdf->HasElement("lidar"))
  {
    return;
  }

  auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
  auto lidar_elem = sdf_mut->GetElement("lidar");
  LoadRange(lidar_elem, model.range, 0.2f, 100.0f);

  if(lidar_elem->HasElement("scan"))
  {
    auto scan_elem = lidar_elem->GetElement("scan");
    if(scan_elem->HasElement("horizontal"))
    {
      LoadScanAxis(scan_elem->GetElement("horizontal"), model.theta);
    }
    if(scan_elem->HasElement("vertical"))
    {
      LoadScanAxis(scan_elem->GetElement("vertical"), model.phi);
    }
  }
}

}  // namespace

SensorModelConfig LoadSensorModelConfig(const std::shared_ptr<const sdf::Element> &_sdf)
{
  SensorModelConfig cfg;

  std::string model_type_str = "spherical";
  if(_sdf && _sdf->HasElement("model_type"))
  {
    model_type_str = ToLower(_sdf->Get<std::string>("model_type"));
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
                << "', falling back to 'spherical'." << std::endl;
    }
    cfg.type = SensorModelType::Spherical;
  }

  // Populate every model's defaults regardless of which is active; keeps
  // each one always in a valid, usable state (harmless: only the active
  // model is ever handed to a simulator).
  LoadSpherical(_sdf, cfg.spherical);
  if(cfg.type == SensorModelType::Pinhole)
  {
    LoadPinhole(_sdf, cfg.pinhole);
  }
  if(cfg.type == SensorModelType::O1Dn)
  {
    LoadO1Dn(_sdf, cfg.o1dn);
  }
  if(cfg.type == SensorModelType::OnDn)
  {
    LoadOnDn(_sdf, cfg.ondn);
  }

  return cfg;
}

}  // namespace rmagine_gazebo_plugins
