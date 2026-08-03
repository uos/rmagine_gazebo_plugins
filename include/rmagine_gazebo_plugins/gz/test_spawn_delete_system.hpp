#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_TEST_SPAWN_DELETE_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_TEST_SPAWN_DELETE_SYSTEM_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace rmagine_gazebo_plugins
{

// Test-only world plugin: on its very first PreUpdate call, creates a
// Visual+Geometry entity and immediately requests its removal in that SAME
// tick, then never does anything again. Regression driver for the
// same-tick spawn+delete case identified during the incremental
// scene-sync rewrite -- gz-sim's "new"/"marked for removal" entity flags
// are independent and both clear at end-of-tick, so this entity appears in
// both EachNew and EachRemoved during the map system's PostUpdate that
// same tick and never again. If the map system processed removals before
// additions (the wrong order), this would leave a permanent zombie
// instance baked into the Embree/OptiX scene forever; see
// rmagine_embree_map_system.cpp's PostUpdate for the fix (additions
// first).
class TestSpawnDeleteSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  TestSpawnDeleteSystem() = default;
  ~TestSpawnDeleteSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  gz::math::Pose3d spawn_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gz::math::Vector3d box_size_{0.5, 0.5, 0.5};
  bool done_{false};
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_TEST_SPAWN_DELETE_SYSTEM_HPP
