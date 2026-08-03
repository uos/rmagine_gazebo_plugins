#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_MAP_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_MAP_SYSTEM_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <chrono>
#include <unordered_set>
#include <unordered_map>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>

#include <sdf/Geometry.hh>

#include <rmagine/map/EmbreeMap.hpp>

namespace rmagine_gazebo_plugins
{

// Scene sync only -- the "map system" half of the map/sensor split
// mirrored from Gazebo Classic's WorldPlugin (scene sync) vs.
// Sensor/SensorPlugin (raycast+publish) separation. gz-sim only has one
// plugin base type (System), so there's no custom Sensor subclass to
// preserve -- this class and RmagineEmbreeSensorSystem, communicating
// through MapRegistry, are the structural analogue of that split.
//
// Keeps ONE persistent EmbreeScene/EmbreeMap for its whole lifetime and
// mutates it in place (add/remove/move geometry, then commit()) instead of
// rebuilding it from scratch on every detected change -- see
// rmagine_embree_map_system.cpp for the incremental sync algorithm.
class RmagineEmbreeMapSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RmagineEmbreeMapSystem() = default;
  ~RmagineEmbreeMapSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  // One entry per entity currently represented in `scene_`. `last_pose` is
  // always the pose actually baked into `geom`'s transform (including the
  // PLANE-normal-rotation adjustment, see BuildVisualInstance) so pose-delta
  // comparisons never need to re-derive it.
  struct TrackedVisual
  {
    rmagine::EmbreeGeometryPtr geom;
    unsigned int scene_geom_id;
    gz::math::Pose3d last_pose;
    std::string geometry_key;
  };

  enum class PrimitiveSceneId
  {
    Box,
    Sphere,
    Cylinder,
    Plane
  };

  void ParseParams(const std::shared_ptr<const sdf::Element> &_sdf);
  bool IsIgnoredVisual(gz::sim::Entity entity,
                       const gz::sim::EntityComponentManager &_ecm) const;
  gz::math::Pose3d ComputeWorldVisualPose(
    gz::sim::Entity entity,
    const gz::sim::EntityComponentManager &_ecm) const;

  // Builds one instance for `geom` at `pose` (mutating `pose` in place for
  // the PLANE case's extra normal-to-+Z rotation, exactly like the pose
  // that ends up baked into the returned geometry's transform) -- shared by
  // the bootstrap pass, EachNew handling, and geometry-shape-change rebuild.
  rmagine::EmbreeGeometryPtr BuildVisualInstance(
    const sdf::Geometry &geom,
    gz::math::Pose3d &pose);

  // Step helpers for the incremental PostUpdate algorithm (see .cpp for the
  // full ordering rationale). All assume `map_mutex_` is already held.
  // Return true iff they actually changed scene topology (added/removed a
  // geometry), so PostUpdate knows whether ObjectEntityMap/MapRegistry's
  // revision need updating.
  bool AddVisual(gz::sim::Entity entity,
                 const gz::sim::EntityComponentManager &_ecm);
  bool RemoveVisual(gz::sim::Entity entity);
  bool SyncGeometryChanges(const gz::sim::EntityComponentManager &_ecm);
  bool SyncPoses(const gz::sim::EntityComponentManager &_ecm);
  bool SyncIgnoreList(const gz::sim::EntityComponentManager &_ecm);
  void RebuildObjectEntityMap();

  rmagine::EmbreeScenePtr PrimitiveScene(PrimitiveSceneId id);

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};

  std::shared_ptr<std::shared_mutex> map_mutex_;
  rmagine::EmbreeMapPtr map_;
  // = map_->scene, kept as its own member since it's accessed far more often
  // than the (rarely touched after construction) EmbreeMap wrapper.
  rmagine::EmbreeScenePtr scene_;

  double update_rate_limit_{200.0};
  double changed_delta_trans_{0.001};
  double changed_delta_rot_{0.001};
  double changed_delta_scale_{0.001};
  bool debug_{false};

  bool map_built_{false};
  std::chrono::nanoseconds last_update_check_{0};
  std::unordered_set<std::string> ignored_model_names_;
  // "model_name::link_name" combined keys -- lets one link of a
  // multi-link model be ignored without ignoring the whole model. Not
  // true self-tagging like Classic's embedded <rmagine_ignore/> (gz-sim's
  // ECS doesn't preserve arbitrary custom SDF tags per-link/model), but
  // achieves the same functional result via the same externally-configured
  // list pattern `ignore_model` above already uses.
  std::unordered_set<std::string> ignored_link_names_;
  std::unordered_map<gz::sim::Entity, TrackedVisual> tracked_visuals_;
  std::unordered_map<PrimitiveSceneId, rmagine::EmbreeScenePtr> primitive_cache_;
  // Mesh-by-URI cache (keyed by "uri:mesh_scale").
  std::unordered_map<std::string, rmagine::EmbreeScenePtr> mesh_cache_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_MAP_SYSTEM_HPP
