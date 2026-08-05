#include "rmagine_gazebo_plugins/gz/test_spawn_delete_system.hpp"

#include "rmagine_gazebo_plugins/gz/gz_compat.hpp"
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Geometry.hh>

#include <sdf/Geometry.hh>
#include <sdf/Box.hh>

namespace rmagine_gazebo_plugins
{

void TestSpawnDeleteSystem::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  world_entity_ = _entity;

  if(!_sdf)
  {
    return;
  }
  if(_sdf->HasElement("pose"))
  {
    spawn_pose_ = _sdf->Get<gz::math::Pose3d>("pose");
  }
  if(_sdf->HasElement("size"))
  {
    box_size_ = _sdf->Get<gz::math::Vector3d>("size");
  }
}

void TestSpawnDeleteSystem::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(done_ || _info.paused)
  {
    return;
  }
  done_ = true;

  const gz::sim::Entity flash = _ecm.CreateEntity();
  _ecm.CreateComponent(flash, gz::sim::components::Pose(spawn_pose_));
  _ecm.CreateComponent(flash, gz::sim::components::ParentEntity(world_entity_));
  _ecm.CreateComponent(flash, gz::sim::components::Name("rmagine_test_flash_visual"));
  _ecm.CreateComponent(flash, gz::sim::components::Visual());

  sdf::Box box_shape;
  box_shape.SetSize(box_size_);
  sdf::Geometry geom;
  geom.SetType(sdf::GeometryType::BOX);
  geom.SetBoxShape(box_shape);
  _ecm.CreateComponent(flash, gz::sim::components::Geometry(geom));

  // Same tick: this entity is simultaneously "new" and "marked for
  // removal" for the rest of this step -- see the class comment.
  _ecm.RequestRemoveEntity(flash);
}

}  // namespace rmagine_gazebo_plugins

RMAGINE_GZ_ADD_PLUGIN(rmagine_gazebo_plugins::TestSpawnDeleteSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::TestSpawnDeleteSystem::ISystemConfigure,
              rmagine_gazebo_plugins::TestSpawnDeleteSystem::ISystemPreUpdate)

RMAGINE_GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::TestSpawnDeleteSystem,
                    "test_spawn_delete_system")
