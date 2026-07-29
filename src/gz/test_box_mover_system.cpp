#include "rmagine_gazebo_plugins/gz/test_box_mover_system.hpp"

#include <cmath>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/PoseCmd.hh>

namespace rmagine_gazebo_plugins
{

void TestBoxMoverSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  model_entity_ = _entity;
  base_pose_ = gz::sim::worldPose(model_entity_, _ecm);
  initialized_ = true;

  if(!_sdf)
  {
    return;
  }

  if(_sdf->HasElement("amplitude"))
  {
    amplitude_ = _sdf->Get<double>("amplitude");
  }
  if(_sdf->HasElement("period"))
  {
    period_ = _sdf->Get<double>("period");
  }
  if(_sdf->HasElement("axis"))
  {
    axis_ = _sdf->Get<gz::math::Vector3d>("axis");
    if(axis_.Length() > 0.0)
    {
      axis_.Normalize();
    }
    else
    {
      axis_.Set(1.0, 0.0, 0.0);
    }
  }
  if(_sdf->HasElement("angular_amplitude"))
  {
    angular_amplitude_ = _sdf->Get<double>("angular_amplitude");
  }
  if(_sdf->HasElement("angular_axis"))
  {
    angular_axis_ = _sdf->Get<gz::math::Vector3d>("angular_axis");
    if(angular_axis_.Length() > 0.0)
    {
      angular_axis_.Normalize();
    }
    else
    {
      angular_axis_.Set(0.0, 0.0, 1.0);
    }
  }
}

void TestBoxMoverSystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused || !initialized_ || period_ <= 0.0)
  {
    return;
  }

  const double t = std::chrono::duration<double>(_info.simTime).count();
  const double phase = 2.0 * GZ_PI * t / period_;
  const double offset = amplitude_ * std::sin(phase);
  const double angle = angular_amplitude_ * std::sin(phase);

  gz::math::Pose3d pose = base_pose_;
  pose.Pos() += axis_ * offset;
  if(angular_amplitude_ != 0.0)
  {
    pose.Rot() = gz::math::Quaterniond(angular_axis_, angle) * pose.Rot();
  }

  auto poseCmd = _ecm.Component<gz::sim::components::WorldPoseCmd>(model_entity_);
  if(!poseCmd)
  {
    _ecm.CreateComponent(model_entity_, gz::sim::components::WorldPoseCmd(pose));
  }
  else
  {
    poseCmd->Data() = pose;
  }
}

}  // namespace rmagine_gazebo_plugins

GZ_ADD_PLUGIN(rmagine_gazebo_plugins::TestBoxMoverSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::TestBoxMoverSystem::ISystemConfigure,
              rmagine_gazebo_plugins::TestBoxMoverSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::TestBoxMoverSystem,
                    "test_box_mover_system")
