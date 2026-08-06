#include "rmagine_gazebo_plugins/gz/rmagine_vulkan_map_system.hpp"

#include "rmagine_gazebo_plugins/gz/gz_compat.hpp"
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

#include <rmagine/map/VulkanMap.hpp>
#include <rmagine/map/vulkan/VulkanScene.hpp>
#include <rmagine/map/vulkan/vulkan_shapes.hpp>
#include <rmagine/map/vulkan/VulkanMesh.hpp>
#include <rmagine/map/vulkan/VulkanInst.hpp>
#include <rmagine/math/linalg.h>
#include <rmagine/map/AssimpIO.hpp>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include "rmagine_gazebo_plugins/gz/vulkan_map_registry.hpp"

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

// Same conversion approach as the Embree/OptiX side
// (rmagine_embree_map_system.cpp's BuildHeightmapEmbreeMesh) -- built on
// the host, then uploaded to the device via VulkanMesh's public
// vertices/faces members.
static constexpr unsigned int kHeightmapMaxGridDim = 100;

static rmagine::VulkanMeshPtr BuildHeightmapVulkanMesh(
  const sdf::Heightmap &heightmap, const std::string &resolved_uri)
{
  gz::common::Image img;
  if(img.Load(resolved_uri) != 0 || !img.Valid())
  {
    std::cerr << "[RmagineVulkanMapSystem] Failed to load heightmap image: "
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

  auto mesh = std::make_shared<rmagine::VulkanMesh>();
  mesh->vertices = verts_cpu;
  mesh->faces = faces_cpu;
  mesh->computeFaceNormals();
  return mesh;
}

// Same fallback rationale as the Embree/OptiX side's
// BuildEmbreeMeshFromGzCommon/BuildOptixMeshFromGzCommon.
static rmagine::VulkanMeshPtr BuildVulkanMeshFromGzCommon(const std::string &resolved_uri)
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

  auto mesh = std::make_shared<rmagine::VulkanMesh>();
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

void RmagineVulkanMapSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  world_entity_ = _entity;
  ParseParams(_sdf);

  map_mutex_ = std::make_shared<std::shared_mutex>();
  VulkanMapRegistry::Instance().SetMapMutex("default", map_mutex_);
}

gz::math::Pose3d RmagineVulkanMapSystem::ComputeWorldVisualPose(
  gz::sim::Entity entity,
  const gz::sim::EntityComponentManager &_ecm) const
{
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
  return pose;
}

rmagine::VulkanGeometryPtr RmagineVulkanMapSystem::BuildVisualInstance(
  const sdf::Geometry &geom,
  gz::math::Pose3d &pose)
{
  rmagine::VulkanGeometryPtr vulkan_geom;

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
      vulkan_geom = instance;
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
      vulkan_geom = instance;
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
      vulkan_geom = instance;
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
      // Same convention as the Embree/OptiX side: VulkanPlane is a flat 1x1
      // unit quad in the XY plane, normal +Z -- an SDF plane's own <normal>
      // (default 0 0 1) is composed onto the entity's world pose as an
      // extra rotation before instantiation. `pose` is mutated here so the
      // caller's tracked `last_pose` always matches the baked transform.
      const gz::math::Vector3d normal = plane->Normal().Normalized();
      gz::math::Quaterniond normal_rot;
      normal_rot.From2Axes(gz::math::Vector3d(0, 0, 1), normal);
      pose = pose * gz::math::Pose3d(gz::math::Vector3d::Zero, normal_rot);
      auto instance = geom_scene->instantiate();
      instance->setScale(rmagine::Vector3{
        static_cast<float>(size.X()), static_cast<float>(size.Y()), 1.0f});
      instance->apply();
      vulkan_geom = instance;
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

      // Mesh-by-URI caching -- same rationale as the Embree/OptiX side. The
      // instance-wrapping every geometry needs anyway (a scene meant to
      // hold multiple placed objects must hold VulkanInst instances, not
      // bare meshes -- see VulkanScene::add()) makes this a natural fit --
      // the cached `mesh_scene` is exactly the thing that already gets
      // instantiated fresh per use.
      const std::string mesh_cache_key = uri + ":" +
        std::to_string(mesh_scale.x) + "," +
        std::to_string(mesh_scale.y) + "," +
        std::to_string(mesh_scale.z);

      rmagine::VulkanScenePtr mesh_scene;
      auto cache_it = mesh_cache_.find(mesh_cache_key);
      if(cache_it != mesh_cache_.end())
      {
        mesh_scene = cache_it->second;
      } else {
        rmagine::VulkanMeshPtr vulkan_mesh;
        rmagine::AssimpIO io;
        const aiScene *ascene = io.ReadFile(uri, 0);
        if(ascene && ascene->HasMeshes())
        {
          vulkan_mesh = rmagine::make_vulkan_mesh(ascene->mMeshes[0]);
        } else {
          vulkan_mesh = BuildVulkanMeshFromGzCommon(uri);
          if(vulkan_mesh && debug_)
          {
            std::cerr << "[RmagineVulkanMapSystem] Assimp failed to parse '" << uri
                      << "', used gz-common's native mesh loader instead." << std::endl;
          }
        }
        if(!vulkan_mesh)
        {
          break;
        }

        vulkan_mesh->setScale(mesh_scale);
        vulkan_mesh->apply();
        vulkan_mesh->commit();

        // Always wrap in a dedicated one-mesh scene + instance rather than
        // adding the raw mesh straight to the top-level scene -- same
        // requirement as the box/sphere/cylinder/plane cases above.
        mesh_scene = vulkan_mesh->makeScene();
        mesh_scene->commit();
        mesh_cache_[mesh_cache_key] = mesh_scene;
      }

      auto instance = mesh_scene->instantiate();
      vulkan_geom = instance;
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
      auto vulkan_mesh = BuildHeightmapVulkanMesh(*heightmap, uri);
      if(!vulkan_mesh)
      {
        break;
      }
      vulkan_mesh->apply();
      vulkan_mesh->commit();

      // Same instance-wrapping requirement as the MESH case just above.
      auto mesh_scene = vulkan_mesh->makeScene();
      mesh_scene->commit();
      auto instance = mesh_scene->instantiate();
      vulkan_geom = instance;
      break;
    }
    default:
      break;
  }

  if(vulkan_geom)
  {
    // setTransform() (not setTransformAndScale()) -- see the identical
    // comment in rmagine_embree_map_system.cpp's BuildVisualInstance for
    // why `scale` isn't a gap: SDF visuals have no scale of their own on
    // Harmonic, only per-shape dimension fields already baked in above.
    rmagine::Transform T = ToRmTransform(pose);
    vulkan_geom->setTransform(T);
    vulkan_geom->apply();
    vulkan_geom->commit();
  }

  return vulkan_geom;
}

bool RmagineVulkanMapSystem::AddVisual(
  gz::sim::Entity entity,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(IsIgnoredVisual(entity, _ecm))
  {
    return false;
  }

  auto geomComp = _ecm.Component<gz::sim::components::Geometry>(entity);
  if(!geomComp)
  {
    return false;
  }

  const sdf::Geometry &geom = geomComp->Data();
  if(!IsSupportedGeometry(geom))
  {
    return false;
  }

  gz::math::Pose3d pose = ComputeWorldVisualPose(entity, _ecm);
  rmagine::VulkanGeometryPtr vulkan_geom = BuildVisualInstance(geom, pose);
  if(!vulkan_geom)
  {
    return false;
  }

  const unsigned int geom_id = scene_->add(vulkan_geom);
  tracked_visuals_[entity] = TrackedVisual{vulkan_geom, geom_id, pose, GeometryKey(geom)};
  return true;
}

bool RmagineVulkanMapSystem::RemoveVisual(gz::sim::Entity entity)
{
  auto it = tracked_visuals_.find(entity);
  if(it == tracked_visuals_.end())
  {
    return false;
  }
  scene_->remove(it->second.scene_geom_id);
  tracked_visuals_.erase(it);
  return true;
}

bool RmagineVulkanMapSystem::SyncGeometryChanges(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  std::vector<gz::sim::Entity> entities;
  entities.reserve(tracked_visuals_.size());
  for(const auto &kv : tracked_visuals_)
  {
    entities.push_back(kv.first);
  }

  for(const auto entity : entities)
  {
    if(_ecm.ComponentState(entity, gz::sim::components::Geometry::typeId)
       == gz::sim::ComponentState::NoChange)
    {
      continue;
    }

    auto geomComp = _ecm.Component<gz::sim::components::Geometry>(entity);
    if(!geomComp)
    {
      continue;
    }

    const sdf::Geometry &geom = geomComp->Data();
    const std::string new_key = GeometryKey(geom);

    auto it = tracked_visuals_.find(entity);
    if(it == tracked_visuals_.end() || it->second.geometry_key == new_key)
    {
      continue;
    }

    scene_->remove(it->second.scene_geom_id);
    tracked_visuals_.erase(it);

    if(!IsSupportedGeometry(geom))
    {
      changed = true;
      continue;
    }

    gz::math::Pose3d pose = ComputeWorldVisualPose(entity, _ecm);
    rmagine::VulkanGeometryPtr vulkan_geom = BuildVisualInstance(geom, pose);
    if(vulkan_geom)
    {
      const unsigned int geom_id = scene_->add(vulkan_geom);
      tracked_visuals_[entity] = TrackedVisual{vulkan_geom, geom_id, pose, new_key};
    }
    changed = true;
  }

  return changed;
}

bool RmagineVulkanMapSystem::SyncPoses(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  for(auto &kv : tracked_visuals_)
  {
    const gz::sim::Entity entity = kv.first;
    TrackedVisual &tracked = kv.second;

    gz::math::Pose3d pose = ComputeWorldVisualPose(entity, _ecm);

    if(auto geomComp = _ecm.Component<gz::sim::components::Geometry>(entity))
    {
      const sdf::Geometry &geom = geomComp->Data();
      if(geom.Type() == sdf::GeometryType::PLANE)
      {
        if(const auto *plane = geom.PlaneShape())
        {
          const gz::math::Vector3d normal = plane->Normal().Normalized();
          gz::math::Quaterniond normal_rot;
          normal_rot.From2Axes(gz::math::Vector3d(0, 0, 1), normal);
          pose = pose * gz::math::Pose3d(gz::math::Vector3d::Zero, normal_rot);
        }
      }
    }

    if(PoseTranslationDelta(tracked.last_pose, pose) <= changed_delta_trans_ &&
       PoseRotationDelta(tracked.last_pose, pose) <= changed_delta_rot_)
    {
      continue;
    }

    tracked.geom->setTransform(ToRmTransform(pose));
    tracked.geom->apply();
    tracked.geom->commit();
    tracked.last_pose = pose;
    changed = true;
  }

  return changed;
}

bool RmagineVulkanMapSystem::SyncIgnoreList(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  std::vector<gz::sim::Entity> entities;
  entities.reserve(tracked_visuals_.size());
  for(const auto &kv : tracked_visuals_)
  {
    entities.push_back(kv.first);
  }

  for(const auto entity : entities)
  {
    if(IsIgnoredVisual(entity, _ecm))
    {
      if(RemoveVisual(entity))
      {
        changed = true;
      }
    }
  }

  return changed;
}

void RmagineVulkanMapSystem::RebuildObjectEntityMap()
{
  auto object_entities = std::make_shared<ObjectEntityMap>();
  for(const auto &kv : tracked_visuals_)
  {
    (*object_entities)[kv.second.scene_geom_id] = kv.first;
  }
  VulkanMapRegistry::Instance().SetObjectEntities("default", object_entities);
}

void RmagineVulkanMapSystem::PostUpdate(
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

  std::unique_lock<std::shared_mutex> lock(*map_mutex_);

  if(!map_built_)
  {
    scene_ = std::make_shared<rmagine::VulkanScene>();
    map_ = std::make_shared<rmagine::VulkanMap>(scene_);

    _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Visual *,
          const gz::sim::components::Geometry *) -> bool
      {
        AddVisual(entity, _ecm);
        return true;
      });

    // See the identical comment in rmagine_embree_map_system.cpp's
    // bootstrap -- `Each` includes entities marked for removal but not yet
    // processed, so a same-tick spawn+delete would otherwise get baked in
    // here and never cleaned up.
    _ecm.EachRemoved<gz::sim::components::Visual, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Visual *,
          const gz::sim::components::Geometry *) -> bool
      {
        RemoveVisual(entity);
        return true;
      });

    try
    {
      scene_->commit();
    }
    catch(const std::exception &e)
    {
      std::cerr << "[RmagineVulkanMapSystem] Failed to commit initial scene: " << e.what() << std::endl;
      return;
    }

    RebuildObjectEntityMap();
    VulkanMapRegistry::Instance().SetVulkanMap("default", map_);
    map_built_ = true;

    if(debug_)
    {
      std::cerr << "[RmagineVulkanMapSystem] Vulkan map bootstrapped with "
                << tracked_visuals_.size() << " supported visuals." << std::endl;
    }
    return;
  }

  bool topology_dirty = false;

  // See rmagine_embree_map_system.cpp's PostUpdate for the full rationale
  // behind this ordering (additions before removals to avoid a same-tick
  // spawn+delete leaving a zombie instance forever).
  _ecm.EachNew<gz::sim::components::Visual, gz::sim::components::Geometry>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Visual *,
        const gz::sim::components::Geometry *) -> bool
    {
      if(AddVisual(entity, _ecm))
      {
        topology_dirty = true;
      }
      return true;
    });

  _ecm.EachRemoved<gz::sim::components::Visual, gz::sim::components::Geometry>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Visual *,
        const gz::sim::components::Geometry *) -> bool
    {
      if(RemoveVisual(entity))
      {
        topology_dirty = true;
      }
      return true;
    });

  if(SyncGeometryChanges(_ecm))
  {
    topology_dirty = true;
  }

  const bool pose_dirty = SyncPoses(_ecm);

  if(SyncIgnoreList(_ecm))
  {
    topology_dirty = true;
  }

  if(topology_dirty || pose_dirty)
  {
    try
    {
      // Adding/removing a visual forces VulkanScene::commit() to fully
      // rebuild the TLAS/BLAS (topology changed); a pose-only change on
      // already-tracked visuals lets it refit the existing acceleration
      // structure in place instead -- see VulkanScene::commit()'s
      // do_update check in rmagine itself. Either way this is ONE commit
      // per tick covering every visual touched above, not one per visual.
      scene_->commit();
    }
    catch(const std::exception &e)
    {
      std::cerr << "[RmagineVulkanMapSystem] Failed to commit scene update: " << e.what() << std::endl;
      return;
    }

    if(topology_dirty)
    {
      RebuildObjectEntityMap();
      // map_ itself is never replaced -- see the identical comment in
      // rmagine_embree_map_system.cpp's PostUpdate.
      VulkanMapRegistry::Instance().SetVulkanMap("default", map_);
    }

    if(debug_)
    {
      std::cerr << "[RmagineVulkanMapSystem] Synced scene (topology_dirty="
                << topology_dirty << ", pose_dirty=" << pose_dirty
                << "), tracking " << tracked_visuals_.size() << " visuals." << std::endl;
    }
  }
}

void RmagineVulkanMapSystem::ParseParams(const std::shared_ptr<const sdf::Element> &_sdf)
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

bool RmagineVulkanMapSystem::IsIgnoredVisual(
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

rmagine::VulkanScenePtr RmagineVulkanMapSystem::PrimitiveScene(PrimitiveSceneId id)
{
  auto it = primitive_cache_.find(id);
  if(it != primitive_cache_.end())
  {
    return it->second;
  }

  rmagine::VulkanGeometryPtr geom;
  switch(id)
  {
    case PrimitiveSceneId::Box:
      geom = std::make_shared<rmagine::VulkanCube>();
      break;
    case PrimitiveSceneId::Sphere:
      geom = std::make_shared<rmagine::VulkanSphere>(30, 30);
      break;
    case PrimitiveSceneId::Cylinder:
      geom = std::make_shared<rmagine::VulkanCylinder>(60);
      break;
    case PrimitiveSceneId::Plane:
      geom = std::make_shared<rmagine::VulkanPlane>();
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

RMAGINE_GZ_ADD_PLUGIN(rmagine_gazebo_plugins::RmagineVulkanMapSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::RmagineVulkanMapSystem::ISystemConfigure,
              rmagine_gazebo_plugins::RmagineVulkanMapSystem::ISystemPostUpdate)

RMAGINE_GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::RmagineVulkanMapSystem,
                    "rmagine_vulkan_map_system")
