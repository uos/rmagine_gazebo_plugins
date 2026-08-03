#include "rmagine_gazebo_plugins/gz/rmagine_embree_map_system.hpp"

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

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/map/embree/EmbreeScene.hpp>
#include <rmagine/map/embree/embree_shapes.h>
#include <rmagine/map/embree/EmbreeMesh.hpp>
#include <rmagine/math/linalg.h>
#include <rmagine/map/AssimpIO.hpp>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include "rmagine_gazebo_plugins/gz/map_registry.hpp"

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

// Builds a triangulated grid mesh from a heightmap's grayscale image (0.0
// black = lowest, 1.0 white = highest). Resolution is capped at
// kHeightmapMaxGridDim per side regardless of the source image's actual
// resolution -- a full-resolution heightmap image would produce an
// intractable triangle count.
static constexpr unsigned int kHeightmapMaxGridDim = 100;

static rmagine::EmbreeGeometryPtr BuildHeightmapEmbreeMesh(
  const sdf::Heightmap &heightmap, const std::string &resolved_uri)
{
  gz::common::Image img;
  if(img.Load(resolved_uri) != 0 || !img.Valid())
  {
    std::cerr << "[RmagineEmbreeMapSystem] Failed to load heightmap image: "
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

  auto mesh = std::make_shared<rmagine::EmbreeMesh>(
    grid_w * grid_h, (grid_w - 1) * (grid_h - 1) * 2);
  auto verts = mesh->vertices();

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
      verts[row * grid_w + col] = rmagine::Vertex{
        static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
  }

  auto faces = mesh->faces();
  size_t f = 0;
  for(unsigned int row = 0; row + 1 < grid_h; row++)
  {
    for(unsigned int col = 0; col + 1 < grid_w; col++)
    {
      const uint32_t i00 = row * grid_w + col;
      const uint32_t i10 = i00 + 1;
      const uint32_t i01 = i00 + grid_w;
      const uint32_t i11 = i01 + 1;
      faces[f++] = rmagine::Face{i00, i10, i11};
      faces[f++] = rmagine::Face{i00, i11, i01};
    }
  }

  mesh->computeFaceNormals();
  mesh->setQuality(RTC_BUILD_QUALITY_LOW);
  mesh->apply();
  mesh->commit();
  return mesh;
}

// Fallback for mesh files Assimp can't parse directly.
static rmagine::EmbreeGeometryPtr BuildEmbreeMeshFromGzCommon(const std::string &resolved_uri)
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

  auto mesh = std::make_shared<rmagine::EmbreeMesh>(n_verts, n_faces);
  auto verts = mesh->vertices();
  for(unsigned int i = 0; i < n_verts; i++)
  {
    const auto v = submesh->Vertex(i);
    verts[i] = rmagine::Vertex{
      static_cast<float>(v.X()), static_cast<float>(v.Y()), static_cast<float>(v.Z())};
  }

  auto faces = mesh->faces();
  for(unsigned int i = 0; i < n_faces; i++)
  {
    faces[i] = rmagine::Face{
      static_cast<uint32_t>(submesh->Index(i * 3 + 0)),
      static_cast<uint32_t>(submesh->Index(i * 3 + 1)),
      static_cast<uint32_t>(submesh->Index(i * 3 + 2))};
  }

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

void RmagineEmbreeMapSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  world_entity_ = _entity;
  ParseParams(_sdf);

  map_mutex_ = std::make_shared<std::shared_mutex>();
  MapRegistry::Instance().SetMapMutex("default", map_mutex_);
}

gz::math::Pose3d RmagineEmbreeMapSystem::ComputeWorldVisualPose(
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

rmagine::EmbreeGeometryPtr RmagineEmbreeMapSystem::BuildVisualInstance(
  const sdf::Geometry &geom,
  gz::math::Pose3d &pose)
{
  rmagine::EmbreeGeometryPtr embree_geom;

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
      embree_geom = instance;
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
      embree_geom = instance;
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
      embree_geom = instance;
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
      // EmbreePlane is a flat 1x1 unit quad in the XY plane (z=0), normal
      // +Z -- SDF planes can specify an arbitrary <normal> (default 0 0 1),
      // so an extra rotation from +Z to that normal is composed onto the
      // entity's own world pose. `pose` is mutated here (not just a local
      // copy) so the caller's tracked `last_pose` always matches what's
      // actually baked into the transform below.
      const gz::math::Vector3d normal = plane->Normal().Normalized();
      gz::math::Quaterniond normal_rot;
      normal_rot.SetFrom2Axes(gz::math::Vector3d(0, 0, 1), normal);
      pose = pose * gz::math::Pose3d(gz::math::Vector3d::Zero, normal_rot);
      auto instance = geom_scene->instantiate();
      instance->setScale(rmagine::Vector3{
        static_cast<float>(size.X()), static_cast<float>(size.Y()), 1.0f});
      instance->apply();
      embree_geom = instance;
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

      // Mesh-by-URI caching, keyed by (uri, mesh_scale): avoids re-reading
      // and re-parsing an unchanged mesh file via Assimp/gz-common every
      // time a new entity referencing it is added, or an existing one is
      // rebuilt after a shape change. A raw (non-instanced) EmbreeGeometry
      // ties its transform to the geometry object itself, so sharing one
      // directly across multiple entities with different poses would
      // silently corrupt whichever was set last -- instancing avoids that.
      const std::string mesh_cache_key = uri + ":" +
        std::to_string(mesh_scale.x) + "," +
        std::to_string(mesh_scale.y) + "," +
        std::to_string(mesh_scale.z);

      rmagine::EmbreeScenePtr mesh_scene;
      auto cache_it = mesh_cache_.find(mesh_cache_key);
      if(cache_it != mesh_cache_.end())
      {
        mesh_scene = cache_it->second;
      } else {
        if(debug_)
        {
          std::cerr << "[RmagineEmbreeMapSystem] Loading mesh from '" << uri
                    << "' (cache miss)." << std::endl;
        }
        std::shared_ptr<rmagine::EmbreeMesh> embree_mesh;
        rmagine::AssimpIO io;
        const aiScene *ascene = io.ReadFile(uri, 0);
        if(ascene && ascene->HasMeshes())
        {
          embree_mesh = std::make_shared<rmagine::EmbreeMesh>(ascene->mMeshes[0]);
        } else {
          auto fallback = BuildEmbreeMeshFromGzCommon(uri);
          embree_mesh = std::dynamic_pointer_cast<rmagine::EmbreeMesh>(fallback);
          if(embree_mesh && debug_)
          {
            std::cerr << "[RmagineEmbreeMapSystem] Assimp failed to parse '" << uri
                      << "', used gz-common's native mesh loader instead." << std::endl;
          }
        }
        if(!embree_mesh)
        {
          break;
        }

        embree_mesh->setScale(mesh_scale);
        embree_mesh->setQuality(RTC_BUILD_QUALITY_LOW);
        embree_mesh->apply();
        embree_mesh->commit();

        mesh_scene = embree_mesh->makeScene();
        mesh_scene->commit();
        mesh_cache_[mesh_cache_key] = mesh_scene;
      }

      auto instance = mesh_scene->instantiate();
      instance->apply();
      embree_geom = instance;
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
      embree_geom = BuildHeightmapEmbreeMesh(*heightmap, uri);
      break;
    }
    default:
      break;
  }

  if(embree_geom)
  {
    // setTransform() (not setTransformAndScale()): SDF's <visual> element
    // has no <scale> concept of its own on Harmonic -- the only per-shape
    // scaling is box/sphere/cylinder/plane's own dimension fields and a
    // mesh's own <geometry><mesh><scale> (both already baked in above via
    // setScale()/mesh_scale). setTransformAndScale() would decompose its
    // Matrix4x4 arg into both m_T and m_S, clobbering the per-shape scale
    // already set back to identity.
    rmagine::Transform T = ToRmTransform(pose);
    embree_geom->setTransform(T);
    embree_geom->setQuality(RTC_BUILD_QUALITY_LOW);
    embree_geom->apply();
    embree_geom->commit();
  }

  return embree_geom;
}

bool RmagineEmbreeMapSystem::AddVisual(
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
  rmagine::EmbreeGeometryPtr embree_geom = BuildVisualInstance(geom, pose);
  if(!embree_geom)
  {
    return false;
  }

  const unsigned int geom_id = scene_->add(embree_geom);
  tracked_visuals_[entity] = TrackedVisual{embree_geom, geom_id, pose, GeometryKey(geom)};
  return true;
}

bool RmagineEmbreeMapSystem::RemoveVisual(gz::sim::Entity entity)
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

bool RmagineEmbreeMapSystem::SyncGeometryChanges(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  // Snapshot the keys first -- rebuilding an entry erases/reinserts into
  // tracked_visuals_, which would invalidate an in-progress iterator.
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

    // Shape actually changed: remove the old instance and rebuild, sampling
    // the pose fresh right now so the rebuilt entry's last_pose reflects
    // this same tick -- SyncPoses() (which runs after this) must see no
    // further delta for an entity whose shape and pose both changed this
    // tick.
    scene_->remove(it->second.scene_geom_id);
    tracked_visuals_.erase(it);

    if(!IsSupportedGeometry(geom))
    {
      changed = true;
      continue;
    }

    gz::math::Pose3d pose = ComputeWorldVisualPose(entity, _ecm);
    rmagine::EmbreeGeometryPtr embree_geom = BuildVisualInstance(geom, pose);
    if(embree_geom)
    {
      const unsigned int geom_id = scene_->add(embree_geom);
      tracked_visuals_[entity] = TrackedVisual{embree_geom, geom_id, pose, new_key};
    }
    changed = true;
  }

  return changed;
}

bool RmagineEmbreeMapSystem::SyncPoses(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  for(auto &kv : tracked_visuals_)
  {
    const gz::sim::Entity entity = kv.first;
    TrackedVisual &tracked = kv.second;

    gz::math::Pose3d pose = ComputeWorldVisualPose(entity, _ecm);

    // PLANE visuals bake an extra normal-to-+Z rotation into the pose that's
    // actually stored (see BuildVisualInstance) -- re-derive the same
    // adjustment here so the comparison against `last_pose` (which already
    // includes it) is apples-to-apples, not a false "pose changed" every
    // single tick.
    if(auto geomComp = _ecm.Component<gz::sim::components::Geometry>(entity))
    {
      const sdf::Geometry &geom = geomComp->Data();
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

bool RmagineEmbreeMapSystem::SyncIgnoreList(
  const gz::sim::EntityComponentManager &_ecm)
{
  bool changed = false;

  // Only re-checks entities already being tracked (an ignored-then-
  // un-ignored entity that was never tracked in the first place would need
  // a full ECM walk to discover -- out of scope, see the design plan).
  // `ignore_model`/`ignore_link` are static plugin SDF params (no runtime
  // reconfigure in gz-sim), so this only ever fires if IsIgnoredVisual's
  // parent-walk result itself changes (e.g. reparenting), not from the
  // ignore lists themselves changing.
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

void RmagineEmbreeMapSystem::RebuildObjectEntityMap()
{
  auto object_entities = std::make_shared<ObjectEntityMap>();
  for(const auto &kv : tracked_visuals_)
  {
    (*object_entities)[kv.second.scene_geom_id] = kv.first;
  }
  MapRegistry::Instance().SetObjectEntities("default", object_entities);
}

void RmagineEmbreeMapSystem::PostUpdate(
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
    // Bootstrap: create the ONE persistent scene this plugin instance will
    // ever use, and populate it with everything already in the world via a
    // single `Each` pass -- `EachNew` alone would miss entities that
    // predate this plugin's first tick (mirrors gz-sim's own
    // RenderUtil::CreateRenderingEntities first-pass pattern).
    scene_ = std::make_shared<rmagine::EmbreeScene>();
    scene_->setQuality(RTC_BUILD_QUALITY_LOW);
    scene_->setFlags(RTC_SCENE_FLAG_DYNAMIC);
    map_ = std::make_shared<rmagine::EmbreeMap>(scene_);

    _ecm.Each<gz::sim::components::Visual, gz::sim::components::Geometry>(
      [&](const gz::sim::Entity &entity,
          const gz::sim::components::Visual *,
          const gz::sim::components::Geometry *) -> bool
      {
        AddVisual(entity, _ecm);
        return true;
      });

    // `Each` (unlike `EachNew`) explicitly includes entities "marked for
    // removal but not yet processed" (see EntityComponentManager.hh) -- an
    // entity created and RequestRemoveEntity()'d within this same first
    // tick (see test_spawn_delete_system.cpp) would otherwise get baked
    // into the scene here and never cleaned up, since this bootstrap path
    // returns before ever reaching the steady-state EachRemoved handling
    // below. Found via the embree_fixture_zombie regression test: its
    // first captured scan briefly read the flash box's occluded range
    // before self-correcting, proving exactly this gap. Removing anything
    // EachRemoved reports, right here, closes it.
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
      std::cerr << "[RmagineEmbreeMapSystem] Failed to commit initial scene: " << e.what() << std::endl;
      return;
    }

    RebuildObjectEntityMap();
    MapRegistry::Instance().SetEmbreeMap("default", map_);
    map_built_ = true;

    if(debug_)
    {
      std::cerr << "[RmagineEmbreeMapSystem] Embree map bootstrapped with "
                << tracked_visuals_.size() << " supported visuals." << std::endl;
    }
    return;
  }

  bool topology_dirty = false;

  // 1. Additions first. gz-sim's "new" and "marked for removal" entity
  // flags are independent and both clear at end-of-tick, so an entity
  // spawned and deleted within the SAME tick appears in both EachNew and
  // EachRemoved that tick and never again. Processing additions before
  // removals turns that into a harmless add-then-immediately-remove
  // instead of a permanent zombie instance baked into the scene forever.
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

  // 2. Removals second.
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

  // 3. Shape/scale changes on already-tracked entities.
  if(SyncGeometryChanges(_ecm))
  {
    topology_dirty = true;
  }

  // 4. Pose updates -- walks only the tracked map, not the whole ECM.
  const bool pose_dirty = SyncPoses(_ecm);

  // 5. Ignore-list re-evaluation (tracked-entities direction only).
  if(SyncIgnoreList(_ecm))
  {
    topology_dirty = true;
  }

  // 6. One batched commit for everything above.
  if(topology_dirty || pose_dirty)
  {
    try
    {
      scene_->commit();
    }
    catch(const std::exception &e)
    {
      std::cerr << "[RmagineEmbreeMapSystem] Failed to commit scene update: " << e.what() << std::endl;
      return;
    }

    if(topology_dirty)
    {
      RebuildObjectEntityMap();
      // map_ itself is never replaced -- this only bumps MapRegistry's
      // revision counter, which RmagineEmbreeSensorSystem uses purely as an
      // observability/"map replaced" signal, not as something that needs to
      // fire on every pose-only edit (sensors share the same EmbreeMapPtr
      // and see mutated positions automatically after commit()).
      MapRegistry::Instance().SetEmbreeMap("default", map_);
    }

    if(debug_)
    {
      std::cerr << "[RmagineEmbreeMapSystem] Synced scene (topology_dirty="
                << topology_dirty << ", pose_dirty=" << pose_dirty
                << "), tracking " << tracked_visuals_.size() << " visuals." << std::endl;
    }
  }
}

void RmagineEmbreeMapSystem::ParseParams(const std::shared_ptr<const sdf::Element> &_sdf)
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

  // "model_name::link_name" -- see the header's comment on
  // ignored_link_names_ for why this isn't true self-tagging like
  // Classic's <rmagine_ignore/>.
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

bool RmagineEmbreeMapSystem::IsIgnoredVisual(
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

rmagine::EmbreeScenePtr RmagineEmbreeMapSystem::PrimitiveScene(PrimitiveSceneId id)
{
  auto it = primitive_cache_.find(id);
  if(it != primitive_cache_.end())
  {
    return it->second;
  }

  rmagine::EmbreeGeometryPtr geom;
  switch(id)
  {
    case PrimitiveSceneId::Box:
      geom = std::make_shared<rmagine::EmbreeCube>();
      break;
    case PrimitiveSceneId::Sphere:
      geom = std::make_shared<rmagine::EmbreeSphere>(30, 30);
      break;
    case PrimitiveSceneId::Cylinder:
      geom = std::make_shared<rmagine::EmbreeCylinder>(60);
      break;
    case PrimitiveSceneId::Plane:
      geom = std::make_shared<rmagine::EmbreePlane>();
      break;
  }

  if(!geom)
  {
    return nullptr;
  }

  geom->setQuality(RTC_BUILD_QUALITY_LOW);
  geom->apply();
  geom->commit();

  auto scene = geom->makeScene();
  scene->setQuality(RTC_BUILD_QUALITY_LOW);
  scene->commit();
  primitive_cache_[id] = scene;
  return scene;
}

}  // namespace rmagine_gazebo_plugins

GZ_ADD_PLUGIN(rmagine_gazebo_plugins::RmagineEmbreeMapSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::RmagineEmbreeMapSystem::ISystemConfigure,
              rmagine_gazebo_plugins::RmagineEmbreeMapSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::RmagineEmbreeMapSystem,
                    "rmagine_embree_map_system")
