#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_MAP_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_MAP_SYSTEM_HPP

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

#include <rmagine/map/OptixMap.hpp>

namespace rmagine_gazebo_plugins
{

// GPU mirror of RmagineEmbreeMapSystem -- see its header comment for the
// architecture rationale and rmagine_optix_map_system.cpp for the
// incremental sync algorithm (identical structure, OptixScene/OptixInst
// API instead of EmbreeScene/EmbreeInstance).
class RmagineOptixMapSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RmagineOptixMapSystem() = default;
  ~RmagineOptixMapSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  struct TrackedVisual
  {
    rmagine::OptixGeometryPtr geom;
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

  rmagine::OptixGeometryPtr BuildVisualInstance(
    const sdf::Geometry &geom,
    gz::math::Pose3d &pose);

  bool AddVisual(gz::sim::Entity entity,
                 const gz::sim::EntityComponentManager &_ecm);
  bool RemoveVisual(gz::sim::Entity entity);
  bool SyncGeometryChanges(const gz::sim::EntityComponentManager &_ecm);
  bool SyncPoses(const gz::sim::EntityComponentManager &_ecm);
  bool SyncIgnoreList(const gz::sim::EntityComponentManager &_ecm);
  void RebuildObjectEntityMap();

  rmagine::OptixScenePtr PrimitiveScene(PrimitiveSceneId id);

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};

  std::shared_ptr<std::shared_mutex> map_mutex_;
  rmagine::OptixMapPtr map_;
  rmagine::OptixScenePtr scene_;

  double update_rate_limit_{200.0};
  double changed_delta_trans_{0.001};
  double changed_delta_rot_{0.001};
  double changed_delta_scale_{0.001};
  bool debug_{false};

  bool map_built_{false};
  std::chrono::nanoseconds last_update_check_{0};
  std::unordered_set<std::string> ignored_model_names_;
  std::unordered_set<std::string> ignored_link_names_;
  std::unordered_map<gz::sim::Entity, TrackedVisual> tracked_visuals_;
  std::unordered_map<PrimitiveSceneId, rmagine::OptixScenePtr> primitive_cache_;
  std::unordered_map<std::string, rmagine::OptixScenePtr> mesh_cache_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_MAP_SYSTEM_HPP
