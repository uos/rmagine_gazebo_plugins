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

#include <rmagine/map/EmbreeMap.hpp>

namespace rmagine_gazebo_plugins
{

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
  struct VisualState
  {
    gz::math::Pose3d pose;
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
  bool HasSceneChanged(const gz::sim::EntityComponentManager &_ecm,
                       std::string *reason = nullptr);
  bool BuildStaticMap(const gz::sim::EntityComponentManager &_ecm);
  rmagine::EmbreeScenePtr PrimitiveScene(PrimitiveSceneId id);

  gz::sim::Entity world_entity_{gz::sim::kNullEntity};

  std::shared_ptr<std::shared_mutex> map_mutex_;
  rmagine::EmbreeMapPtr map_;

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
  // ECS doesn't preserve arbitrary custom SDF tags per-link/model --
  // Link/Model/Visual components are all empty markers, confirmed by
  // checking their component data types), but achieves the same
  // functional result via the same externally-configured-list pattern
  // `ignore_model` above already uses.
  std::unordered_set<std::string> ignored_link_names_;
  std::unordered_map<gz::sim::Entity, VisualState> visual_states_;
  std::unordered_map<PrimitiveSceneId, rmagine::EmbreeScenePtr> primitive_cache_;
  // Mesh-by-URI cache (keyed by "uri:mesh_scale") -- see BuildStaticMap's
  // MESH case comment for why this exists and how it stays correct when
  // multiple entities share one mesh file.
  std::unordered_map<std::string, rmagine::EmbreeScenePtr> mesh_cache_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_MAP_SYSTEM_HPP
