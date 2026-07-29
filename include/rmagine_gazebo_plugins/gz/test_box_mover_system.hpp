#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_TEST_BOX_MOVER_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_TEST_BOX_MOVER_SYSTEM_HPP

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

namespace rmagine_gazebo_plugins
{

class TestBoxMoverSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  TestBoxMoverSystem() = default;
  ~TestBoxMoverSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::math::Pose3d base_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  gz::math::Vector3d axis_{1.0, 0.0, 0.0};
  double amplitude_{1.0};
  double period_{6.0};

  // Angular oscillation, independent of the linear one above (same period,
  // its own axis/amplitude). Defaults to zero amplitude so existing worlds
  // that only set axis/amplitude/period are unaffected -- this is additive,
  // not a behavior change to the translation-only path.
  gz::math::Vector3d angular_axis_{0.0, 0.0, 1.0};
  double angular_amplitude_{0.0};

  bool initialized_{false};
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_TEST_BOX_MOVER_SYSTEM_HPP
