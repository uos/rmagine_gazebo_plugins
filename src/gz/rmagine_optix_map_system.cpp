#include "rmagine_gazebo_plugins/gz/rmagine_optix_map_system.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Plane.hh>
#include <sdf/Mesh.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Filesystem.hh>

#include <gz/common/Image.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/MeshManager.hh>

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/map/optix/OptixScene.hpp>
#include <rmagine/map/optix/optix_shapes.h>
#include <rmagine/map/optix/OptixMesh.hpp>
#include <rmagine/map/optix/OptixInst.hpp>
#include <rmagine/math/linalg.h>
#include <rmagine/map/AssimpIO.hpp>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include "rmagine_gazebo_plugins/gz/optix_map_registry.hpp"

namespace rmagine_gazebo_plugins
{

static rmagine::Transform ToRmTransform(const gz::math::Pose3d &pose)
{
  rmagine::Transform T;
  T.R.x = pose.Rot().X();
  T.R.y = pose.Rot().Y();
  T.R.z = pose.Rot().Z();
  T.R.w = pose.Rot().W();
  T.t.x = pose.Pos().X();
  T.t.y = pose.Pos().Y();
  T.t.z = pose.Pos().Z();
  return T;
}

static rmagine::Vector3 ToRmVector3(const gz::math::Vector3d &v)
{
  rmagine::Vector3 r;
  r.x = v.X();
  r.y = v.Y();
  r.z = v.Z();
  return r;
}

static std::string ResolveMeshUri(const std::string &uri)
{
  std::string resolved = sdf::findFile(uri);
  if(resolved.empty())
  {
    resolved = uri;
  }
  return resolved;
}

// Same conversion approach as the Embree side
// (rmagine_embree_map_system.cpp's BuildHeightmapEmbreeMesh -- see its
// comment for why gz::common::Image is used instead of porting Classic's
// gazebo::common::HeightmapDataLoader-based code) -- built on the host,
// then uploaded to VRAM via OptixMesh's public vertices/faces members
// (Memory<T, VRAM_CUDA>, matching make_optix_mesh()'s own upload
// pattern in OptixMesh.cpp).
static constexpr unsigned int kHeightmapMaxGridDim = 100;

static rmagine::OptixMeshPtr BuildHeightmapOptixMesh(
  const sdf::Heightmap &heightmap, const std::string &resolved_uri)
{
  gz::common::Image img;
  if(img.Load(resolved_uri) != 0 || !img.Valid())
  {
    std::cerr << "[RmagineOptixMapSystem] Failed to load heightmap image: "
              << resolved_uri << std::endl;
    return nullptr;
  }

  const unsigned int img_w = img.Width();
  const unsigned int img_h = img.Height();
  if(img_w < 2 || img_h < 2)
  {
    return nullptr;
  }

  const unsigned int grid_w = std::min(img_w, kHeightmapMaxGridDim);
  const unsigned int grid_h = std::min(img_h, kHeightmapMaxGridDim);

  const gz::math::Vector3d size = heightmap.Size();
  const gz::math::Vector3d pos = heightmap.Position();

  rmagine::Memory<rmagine::Point, rmagine::RAM> verts_cpu(grid_w * grid_h);
  for(unsigned int row = 0; row < grid_h; row++)
  {
    const unsigned int src_y = row * (img_h - 1) / (grid_h - 1);
    const double y = -size.Y() / 2.0 + static_cast<double>(row) * size.Y() / (grid_h - 1) + pos.Y();
    for(unsigned int col = 0; col < grid_w; col++)
    {
      const unsigned int src_x = col * (img_w - 1) / (grid_w - 1);
      const double x = -size.X() / 2.0 + static_cast<double>(col) * size.X() / (grid_w - 1) + pos.X();
      const float height = img.Pixel(src_x, src_y).R();
      const double z = static_cast<double>(height) * size.Z() + pos.Z();
      verts_cpu[row * grid_w + col] = rmagine::Point{
        static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
  }

  rmagine::Memory<rmagine::Face, rmagine::RAM> faces_cpu((grid_w - 1) * (grid_h - 1) * 2);
  size_t f = 0;
  for(unsigned int row = 0; row + 1 < grid_h; row++)
  {
    for(unsigned int col = 0; col + 1 < grid_w; col++)
    {
      const uint32_t i00 = row * grid_w + col;
      const uint32_t i10 = i00 + 1;
      const uint32_t i01 = i00 + grid_w;
      const uint32_t i11 = i01 + 1;
      faces_cpu[f++] = rmagine::Face{i00, i10, i11};
      faces_cpu[f++] = rmagine::Face{i00, i11, i01};
    }
  }

  auto mesh = std::make_shared<rmagine::OptixMesh>();
  mesh->vertices = verts_cpu;
  mesh->faces = faces_cpu;
  mesh->computeFaceNormals();
  return mesh;
}

// Same fallback rationale as the Embree side's
// BuildEmbreeMeshFromGzCommon (see its comment) -- gz-common's own
// native mesh loader, used only when Assimp itself fails to parse a
// mesh file.
static rmagine::OptixMeshPtr BuildOptixMeshFromGzCommon(const std::string &resolved_uri)
{
  auto *gz_mesh = gz::common::MeshManager::Instance()->Load(resolved_uri);
  if(!gz_mesh || gz_mesh->SubMeshCount() == 0)
  {
    return nullptr;
  }

  auto submesh = gz_mesh->SubMeshByIndex(0).lock();
  if(!submesh || submesh->VertexCount() == 0 || submesh->IndexCount() == 0)
  {
    return nullptr;
  }

  const unsigned int n_verts = submesh->VertexCount();
  const unsigned int n_faces = submesh->IndexCount() / 3;

  rmagine::Memory<rmagine::Point, rmagine::RAM> verts_cpu(n_verts);
  for(unsigned int i = 0; i < n_verts; i++)
  {
    const auto v = submesh->Vertex(i);
    verts_cpu[i] = rmagine::Point{
      static_cast<float>(v.X()), static_cast<float>(v.Y()), static_cast<float>(v.Z())};
  }

  rmagine::Memory<rmagine::Face, rmagine::RAM> faces_cpu(n_faces);
  for(unsigned int i = 0; i < n_faces; i++)
  {
    faces_cpu[i] = rmagine::Face{
      static_cast<uint32_t>(submesh->Index(i * 3 + 0)),
      static_cast<uint32_t>(submesh->Index(i * 3 + 1)),
      static_cast<uint32_t>(submesh->Index(i * 3 + 2))};
  }

  auto mesh = std::make_shared<rmagine::OptixMesh>();
  mesh->vertices = verts_cpu;
  mesh->faces = faces_cpu;
  mesh->computeFaceNormals();
  return mesh;
}

static bool IsSupportedGeometry(const sdf::Geometry &geom)
{
  switch(geom.Type())
  {
    case sdf::GeometryType::BOX:
    case sdf::GeometryType::SPHERE:
    case sdf::GeometryType::CYLINDER:
    case sdf::GeometryType::MESH:
    case sdf::GeometryType::PLANE:
    case sdf::GeometryType::HEIGHTMAP:
      return true;
    default:
      return false;
  }
}

static std::string GeometryKey(const sdf::Geometry &geom)
{
  std::ostringstream out;
  out << std::setprecision(9) << static_cast<int>(geom.Type()) << ":";
  switch(geom.Type())
  {
    case sdf::GeometryType::BOX:
    {
      const auto *box = geom.BoxShape();
      if(box)
      {
        const auto size = box->Size();
        out << size.X() << "," << size.Y() << "," << size.Z();
      }
      break;
    }
    case sdf::GeometryType::SPHERE:
    {
      const auto *sphere = geom.SphereShape();
      if(sphere)
      {
        out << sphere->Radius();
      }
      break;
    }
    case sdf::GeometryType::CYLINDER:
    {
      const auto *cyl = geom.CylinderShape();
      if(cyl)
      {
        out << cyl->Radius() << "," << cyl->Length();
      }
      break;
    }
    case sdf::GeometryType::MESH:
    {
      const auto *mesh = geom.MeshShape();
      if(mesh)
      {
        const auto scale = mesh->Scale();
        out << mesh->Uri() << ":"
            << scale.X() << "," << scale.Y() << "," << scale.Z();
      }
      break;
    }
    case sdf::GeometryType::PLANE:
    {
      const auto *plane = geom.PlaneShape();
      if(plane)
      {
        const auto size = plane->Size();
        const auto normal = plane->Normal();
        out << size.X() << "," << size.Y() << ":"
            << normal.X() << "," << normal.Y() << "," << normal.Z();
      }
      break;
    }
    case sdf::GeometryType::HEIGHTMAP:
    {
      const auto *heightmap = geom.HeightmapShape();
      if(heightmap)
      {
        const auto size = heightmap->Size();
        const auto pos = heightmap->Position();
        out << heightmap->Uri() << ":"
            << size.X() << "," << size.Y() << "," << size.Z() << ":"
            << pos.X() << "," << pos.Y() << "," << pos.Z();
      }
      break;
    }
    default:
      out << "unsupported";
      break;
  }
  return out.str();
}

static double PoseTranslationDelta(const gz::math::Pose3d &a, const gz::math::Pose3d &b)
{
  return (a.Pos() - b.Pos()).Length();
}

static double PoseRotationDelta(const gz::math::Pose3d &a, const gz::math::Pose3d &b)
{
  const auto qa = a.Rot();
  const auto qb = b.Rot();
  const double dot = std::abs(
    qa.W() * qb.W() +
    qa.X() * qb.X() +
    qa.Y() * qb.Y() +
    qa.Z() * qb.Z());
  const double clamped = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(clamped);
}

void RmagineOptixMapSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  world_entity_ = _entity;
  ParseParams(_sdf);

  map_mutex_ = std::make_shared<std::shared_mutex>();
}

void RmagineOptixMapSystem::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused)
  {
    return;
  }

  const auto sim_now = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_limit_));

  if(map_built_ && (sim_now - last_update_check_) < period)
  {
    return;
  }

  last_update_check_ = sim_now;

  if(!map_built_)
  {
    map_built_ = BuildStaticMap(_ecm);
    return;
  }

  std::string reason;
  if(HasSceneChanged(_ecm, &reason))
  {
    if(debug_)
    {
      std::cerr << "[RmagineOptixMapSystem] Scene change detected (" << reason
                << "). Rebuilding OptiX map." << std::endl;
    }
    map_built_ = BuildStaticMap(_ecm);
  }
}

void RmagineOptixMapSystem::ParseParams(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if(!_sdf)
  {
    return;
  }

  sdf::ElementPtr ignoreElem;
  if(_sdf->HasElement("ignore_model"))
  {
    auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
    ignoreElem = sdf_mut->GetElement("ignore_model");
  }

  while(ignoreElem)
  {
    ignored_model_names_.insert(ignoreElem->Get<std::string>());
    ignoreElem = ignoreElem->GetNextElement("ignore_model");
  }

  sdf::ElementPtr ignoreLinkElem;
  if(_sdf->HasElement("ignore_link"))
  {
    auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
    ignoreLinkElem = sdf_mut->GetElement("ignore_link");
  }

  while(ignoreLinkElem)
  {
    ignored_link_names_.insert(ignoreLinkElem->Get<std::string>());
    ignoreLinkElem = ignoreLinkElem->GetNextElement("ignore_link");
  }

  if(_sdf->HasElement("update"))
  {
    auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
    auto updateElem = sdf_mut->GetElement("update");
    if(updateElem->HasElement("rate_limit"))
    {
      update_rate_limit_ = updateElem->Get<double>("rate_limit");
    }
    if(updateElem->HasElement("delta_trans"))
    {
      changed_delta_trans_ = updateElem->Get<double>("delta_trans");
    }
    if(updateElem->HasElement("delta_rot"))
    {
      changed_delta_rot_ = updateElem->Get<double>("delta_rot");
    }
    if(updateElem->HasElement("delta_scale"))
    {
      changed_delta_scale_ = updateElem->Get<double>("delta_scale");
    }
  }

  if(_sdf->HasElement("debug"))
  {
    debug_ = _sdf->Get<bool>("debug");
  }
}

bool RmagineOptixMapSystem::IsIgnoredVisual(
  gz::sim::Entity entity,
  const gz::sim::EntityComponentManager &_ecm) const
{
  auto current = entity;
  std::string link_name;
  while(current != gz::sim::kNullEntity)
  {
    if(_ecm.Component<gz::sim::components::Link>(current))
    {
      if(auto nameComp = _ecm.Component<gz::sim::components::Name>(current))
      {
        link_name = nameComp->Data();
      }
    }

    if(_ecm.Component<gz::sim::components::Model>(current))
    {
      if(auto nameComp = _ecm.Component<gz::sim::components::Name>(current))
      {
        if(ignored_model_names_.count(nameComp->Data()) > 0)
        {
          return true;
        }
        if(!link_name.empty() && !ignored_link_names_.empty())
        {
          const std::string combined = nameComp->Data() + "::" + link_name;
          if(ignored_link_names_.count(combined) > 0)
          {
            return true;
          }
        }
      }
    }

    auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(current);
    if(!parentComp)
    {
      break;
    }
    current = parentComp->Data();
  }
  return false;
}

bool RmagineOptixMapSystem::BuildStaticMap(const gz::sim::EntityComponentManager &_ecm)
{
  std::unique_lock<std::shared_mutex> lock(*map_mutex_);
  auto new_scene = std::make_shared<rmagine::OptixScene>();
  std::unordered_map<gz::sim::Entity, VisualState> new_visual_states;
  auto new_object_entities = std::make_shared<ObjectEntityMap>();

  _ecm.Each<gz::sim::components::Visual,
            gz::sim::components::Geometry>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Visual *,
        const gz::sim::components::Geometry *geomComp) -> bool
    {
      if(!geomComp)
      {
        return true;
      }

      if(IsIgnoredVisual(entity, _ecm))
      {
        return true;
      }

      const sdf::Geometry &geom = geomComp->Data();
      if(!IsSupportedGeometry(geom))
      {
        return true;
      }
      const std::string geometry_key = GeometryKey(geom);
      gz::math::Pose3d pose = gz::sim::worldPose(entity, _ecm);
      if(auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(entity))
      {
        const auto parentEntity = parentComp->Data();
        gz::math::Pose3d parentPose = gz::sim::worldPose(parentEntity, _ecm);
        if(auto localPoseComp = _ecm.Component<gz::sim::components::Pose>(entity))
        {
          pose = parentPose * localPoseComp->Data();
        }
      }
      gz::math::Vector3d scale(1.0, 1.0, 1.0);

      rmagine::OptixGeometryPtr optix_geom;

      switch(geom.Type())
      {
        case sdf::GeometryType::BOX:
        {
          const auto *box = geom.BoxShape();
          if(!box)
          {
            break;
          }
          auto geom_scene = PrimitiveScene(PrimitiveSceneId::Box);
          if(!geom_scene)
          {
            break;
          }
          auto instance = geom_scene->instantiate();
          instance->setScale(ToRmVector3(box->Size()));
          instance->apply();
          optix_geom = instance;
          break;
        }
        case sdf::GeometryType::SPHERE:
        {
          const auto *sphere = geom.SphereShape();
          if(!sphere)
          {
            break;
          }
          auto geom_scene = PrimitiveScene(PrimitiveSceneId::Sphere);
          if(!geom_scene)
          {
            break;
          }
          const float diameter = static_cast<float>(sphere->Radius() * 2.0);
          auto instance = geom_scene->instantiate();
          instance->setScale(rmagine::Vector3{diameter, diameter, diameter});
          instance->apply();
          optix_geom = instance;
          break;
        }
        case sdf::GeometryType::CYLINDER:
        {
          const auto *cyl = geom.CylinderShape();
          if(!cyl)
          {
            break;
          }
          auto geom_scene = PrimitiveScene(PrimitiveSceneId::Cylinder);
          if(!geom_scene)
          {
            break;
          }
          const float diameter = static_cast<float>(cyl->Radius() * 2.0);
          const float length = static_cast<float>(cyl->Length());
          auto instance = geom_scene->instantiate();
          instance->setScale(rmagine::Vector3{diameter, diameter, length});
          instance->apply();
          optix_geom = instance;
          break;
        }
        case sdf::GeometryType::PLANE:
        {
          const auto *plane = geom.PlaneShape();
          if(!plane)
          {
            break;
          }
          auto geom_scene = PrimitiveScene(PrimitiveSceneId::Plane);
          if(!geom_scene)
          {
            break;
          }
          const auto size = plane->Size();
          // Same convention as the Embree side (rmagine_embree_map_system.cpp):
          // OptixPlane is a flat 1x1 unit quad in the XY plane, normal +Z --
          // an SDF plane's own <normal> (default 0 0 1) is composed onto the
          // entity's world pose as an extra rotation before instantiation.
          const gz::math::Vector3d normal = plane->Normal().Normalized();
          gz::math::Quaterniond normal_rot;
          normal_rot.SetFrom2Axes(gz::math::Vector3d(0, 0, 1), normal);
          pose = pose * gz::math::Pose3d(gz::math::Vector3d::Zero, normal_rot);
          auto instance = geom_scene->instantiate();
          instance->setScale(rmagine::Vector3{
            static_cast<float>(size.X()), static_cast<float>(size.Y()), 1.0f});
          instance->apply();
          optix_geom = instance;
          break;
        }
        case sdf::GeometryType::MESH:
        {
          const auto *mesh = geom.MeshShape();
          if(!mesh)
          {
            break;
          }
          const std::string uri = ResolveMeshUri(mesh->Uri());
          if(uri.empty())
          {
            break;
          }
          rmagine::Vector3 mesh_scale = ToRmVector3(mesh->Scale());
          rmagine::Vector3 world_scale = ToRmVector3(scale);
          rmagine::Vector3 total_scale = {mesh_scale.x * world_scale.x,
                                          mesh_scale.y * world_scale.y,
                                          mesh_scale.z * world_scale.z};

          // Mesh-by-URI caching -- same rationale as the Embree side (see
          // rmagine_embree_map_system.cpp's identical MESH case comment,
          // including why any already-cached entry is always safe to
          // reuse with no per-pass first-use guard: BuildStaticMap() runs
          // single-threaded, and an earlier guarded version of this code
          // only let the FIRST entity per pass sharing a cached mesh get
          // a hit, silently reloading every other one from disk every
          // single pass -- found via a real regression test, fixed on
          // both backends). The instance-wrapping this already needs for
          // the cudaErrorIllegalAddress fix below makes this a natural
          // fit -- the cached `mesh_scene` is exactly the thing that
          // already gets instantiated fresh per use.
          const std::string mesh_cache_key = uri + ":" +
            std::to_string(mesh_scale.x) + "," +
            std::to_string(mesh_scale.y) + "," +
            std::to_string(mesh_scale.z);

          rmagine::OptixScenePtr mesh_scene;
          auto cache_it = mesh_cache_.find(mesh_cache_key);
          if(cache_it != mesh_cache_.end())
          {
            mesh_scene = cache_it->second;
          } else {
            rmagine::OptixMeshPtr optix_mesh;
            rmagine::AssimpIO io;
            const aiScene *ascene = io.ReadFile(uri, 0);
            if(ascene && ascene->HasMeshes())
            {
              optix_mesh = rmagine::make_optix_mesh(ascene->mMeshes[0]);
            } else {
              optix_mesh = BuildOptixMeshFromGzCommon(uri);
              if(optix_mesh && debug_)
              {
                std::cerr << "[RmagineOptixMapSystem] Assimp failed to parse '" << uri
                          << "', used gz-common's native mesh loader instead." << std::endl;
              }
            }
            if(!optix_mesh)
            {
              break;
            }

            optix_mesh->setScale(total_scale);
            optix_mesh->apply();
            optix_mesh->commit();

            // A raw OptixMesh added directly to the top-level scene (no
            // instance wrapping) causes a downstream cudaErrorIllegalAddress
            // the first time a simulator reads back results from it -- root
            // caused with a standalone repro outside gz-sim (mirrors this
            // exact construction, no gz-sim/cross-.so involvement at all):
            // simulate()/move_waves()/signal_shader() all run and sync
            // cleanly, the corruption only surfaces on the next device-to-
            // host copy. rmagine's own `make_optix_scene()` (used by
            // `import_optix_map()`, which never hit this) always wraps
            // meshes in an OptixInst instead; box/sphere/cylinder below
            // already do the same via `geom_scene->instantiate()`. This was
            // simply the first time the MESH case itself got run end-to-end
            // (prior GPU milestones only exercised box geometry). Matching
            // that pattern here fixes it -- verified with the same repro.
            mesh_scene = optix_mesh->makeScene();
            mesh_scene->commit();
            mesh_cache_[mesh_cache_key] = mesh_scene;
          }

          auto instance = mesh_scene->instantiate();
          optix_geom = instance;
          break;
        }
        case sdf::GeometryType::HEIGHTMAP:
        {
          const auto *heightmap = geom.HeightmapShape();
          if(!heightmap)
          {
            break;
          }
          const std::string uri = ResolveMeshUri(heightmap->Uri());
          if(uri.empty())
          {
            break;
          }
          auto optix_mesh = BuildHeightmapOptixMesh(*heightmap, uri);
          if(!optix_mesh)
          {
            break;
          }
          optix_mesh->apply();
          optix_mesh->commit();

          // Same instance-wrapping requirement as the MESH case just
          // above (see its comment) -- a raw OptixMesh added directly
          // crashes on readback.
          auto mesh_scene = optix_mesh->makeScene();
          mesh_scene->commit();
          auto instance = mesh_scene->instantiate();
          optix_geom = instance;
          break;
        }
        default:
          break;
      }

      if(optix_geom)
      {
        // setTransform() (not setTransformAndScale()) deliberately: `scale`
        // above is always identity -- unlike Gazebo Classic's
        // gazebo::msgs::Visual, which had its own scale field, SDF's
        // <visual> element (checked against sdformat's own visual.sdf
        // schema across every format version, and sdf::Visual's C++ API)
        // has no <scale> concept at all in Harmonic/gz-sim; the only
        // per-shape scaling is box/sphere/cylinder/plane's own dimension
        // fields and a mesh's own <geometry><mesh><scale> (both already
        // handled below). So `scale` isn't a gap to fix, just a variable
        // that's structurally always identity -- kept only because
        // setTransformAndScale() decomposes its Matrix4x4 arg
        // into BOTH m_T and m_S -- using it here would silently clobber the
        // per-shape scale already set above (box->Size(), sphere/cylinder
        // diameter, or the mesh's baked total_scale) back to identity.
        // Confirmed via a standalone repro against this exact API: a 0.2
        // box and a 1.0 box raytraced to the identical hit distance until
        // this was changed to setTransform(). See MIGRATION_HANDOFF.md.
        rmagine::Transform T = ToRmTransform(pose);
        optix_geom->setTransform(T);
        optix_geom->apply();
        optix_geom->commit();
        const unsigned int geom_id = new_scene->add(optix_geom);
        (*new_object_entities)[geom_id] = entity;
        new_visual_states.emplace(entity, VisualState{pose, geometry_key});
      }

      return true;
    });

  try
  {
    new_scene->commit();
  }
  catch(const std::exception &e)
  {
    std::cerr << "[RmagineOptixMapSystem] Failed to commit scene: " << e.what() << std::endl;
    return false;
  }

  auto new_map = std::make_shared<rmagine::OptixMap>(new_scene);
  map_ = new_map;
  visual_states_ = std::move(new_visual_states);
  OptixMapRegistry::Instance().SetOptixMap("default", new_map);
  OptixMapRegistry::Instance().SetObjectEntities("default", new_object_entities);
  if(debug_)
  {
    std::cerr << "[RmagineOptixMapSystem] OptiX map rebuilt with "
              << visual_states_.size() << " supported visuals and published at revision "
              << OptixMapRegistry::Instance().GetOptixMapRevision("default") << "." << std::endl;
  }
  return true;
}

bool RmagineOptixMapSystem::HasSceneChanged(
  const gz::sim::EntityComponentManager &_ecm,
  std::string *reason)
{
  std::unordered_map<gz::sim::Entity, VisualState> current_states;
  bool changed = false;

  _ecm.Each<gz::sim::components::Visual,
            gz::sim::components::Geometry>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Visual *,
        const gz::sim::components::Geometry *geomComp) -> bool
    {
      if(!geomComp)
      {
        return true;
      }

      if(IsIgnoredVisual(entity, _ecm))
      {
        return true;
      }

      const sdf::Geometry &geom = geomComp->Data();
      if(!IsSupportedGeometry(geom))
      {
        return true;
      }
      gz::math::Pose3d pose = gz::sim::worldPose(entity, _ecm);
      if(auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(entity))
      {
        const auto parentEntity = parentComp->Data();
        gz::math::Pose3d parentPose = gz::sim::worldPose(parentEntity, _ecm);
        if(auto localPoseComp = _ecm.Component<gz::sim::components::Pose>(entity))
        {
          pose = parentPose * localPoseComp->Data();
        }
      }
      // Must match BuildStaticMap()'s PLANE case exactly -- see the
      // identical comment in rmagine_embree_map_system.cpp's
      // HasSceneChanged() for why (a real infinite-rebuild-loop bug found
      // via a real vertical-wall plane test, fixed there first).
      if(geom.Type() == sdf::GeometryType::PLANE)
      {
        if(const auto *plane = geom.PlaneShape())
        {
          const gz::math::Vector3d normal = plane->Normal().Normalized();
          gz::math::Quaterniond normal_rot;
          normal_rot.SetFrom2Axes(gz::math::Vector3d(0, 0, 1), normal);
          pose = pose * gz::math::Pose3d(gz::math::Vector3d::Zero, normal_rot);
        }
      }

      VisualState state{pose, GeometryKey(geom)};
      current_states.emplace(entity, state);

      auto it = visual_states_.find(entity);
      if(it == visual_states_.end())
      {
        if(reason)
        {
          *reason = "added visual";
        }
        changed = true;
        return true;
      }

      if(it->second.geometry_key != state.geometry_key)
      {
        if(reason)
        {
          *reason = "geometry changed";
        }
        changed = true;
        return true;
      }

      if(PoseTranslationDelta(it->second.pose, state.pose) > changed_delta_trans_)
      {
        if(reason)
        {
          *reason = "pose changed";
        }
        changed = true;
        return true;
      }

      if(PoseRotationDelta(it->second.pose, state.pose) > changed_delta_rot_)
      {
        if(reason)
        {
          *reason = "pose changed";
        }
        changed = true;
        return true;
      }

      return true;
    });

  if(changed)
  {
    return true;
  }

  if(current_states.size() != visual_states_.size())
  {
    if(reason)
    {
      *reason = "removed visual";
    }
    return true;
  }

  return false;
}

rmagine::OptixScenePtr RmagineOptixMapSystem::PrimitiveScene(PrimitiveSceneId id)
{
  auto it = primitive_cache_.find(id);
  if(it != primitive_cache_.end())
  {
    return it->second;
  }

  rmagine::OptixGeometryPtr geom;
  switch(id)
  {
    case PrimitiveSceneId::Box:
      geom = std::make_shared<rmagine::OptixCube>();
      break;
    case PrimitiveSceneId::Sphere:
      geom = std::make_shared<rmagine::OptixSphere>(30, 30);
      break;
    case PrimitiveSceneId::Cylinder:
      geom = std::make_shared<rmagine::OptixCylinder>(60);
      break;
    case PrimitiveSceneId::Plane:
      geom = std::make_shared<rmagine::OptixPlane>();
      break;
  }

  if(!geom)
  {
    return nullptr;
  }

  geom->apply();
  geom->commit();

  auto scene = geom->makeScene();
  scene->commit();
  primitive_cache_[id] = scene;
  return scene;
}

}  // namespace rmagine_gazebo_plugins

GZ_ADD_PLUGIN(rmagine_gazebo_plugins::RmagineOptixMapSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::RmagineOptixMapSystem::ISystemConfigure,
              rmagine_gazebo_plugins::RmagineOptixMapSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::RmagineOptixMapSystem,
                    "rmagine_optix_map_system")
