#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_SENSOR_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_SENSOR_SYSTEM_HPP

#include <memory>
#include <shared_mutex>
#include <string>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/types/sensor_models.h>

#include "rmagine_gazebo_plugins/gz/sensor_model_config.hpp"

namespace rmagine_gazebo_plugins
{

// One of these is created per discovered `<sensor type="custom"
// gz:type="rmagine_embree">` entity, owned by RmagineEmbreeSensorSystem --
// see that class's comment for why. Holds everything a per-sensor plugin
// instance used to hold; the gz-transport node is shared across all
// instances now (owned by the factory) rather than one per sensor. Publishes
// only gz-native messages (gz::msgs::LaserScan/PointCloudPacked) -- no ROS
// dependency at all. TF is NOT published here: every sensor frame in this
// package's worlds is a fixed joint, so gz-sim's own
// gz::sim::systems::PosePublisher system (attached to the robot/model in
// SDF) plus ros_gz_bridge's Pose_V->TFMessage conversion covers it
// completely -- see README.md and worlds/gz_embree_robot_demo.sdf.
class RmagineEmbreeSensorInstance
{
public:
  // Parses this sensor's own SDF element (`sdf::Sensor::Element()` --
  // i.e. the <sensor> element itself, since this is a "custom" sensor type,
  // not a nested <plugin>'s SDF) and creates this instance's publishers on
  // the shared gz-transport node.
  void Load(
    gz::sim::Entity sensor_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::transport::Node *gz_node);

  void Update(const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm);

private:
  void RefreshSimulator();
  void ResolveFrameEntity(const gz::sim::EntityComponentManager &_ecm);

  // Runs setTsb/setModel/simulate for whichever (simulator, model) pair is
  // currently active, then publishes PointCloudPacked (and LaserScan, for
  // Spherical) via the shared templated helpers in sensor_model_publish.hpp.
  template<typename SimPtrT, typename ModelT>
  void RunAndPublish(
    SimPtrT &sim,
    const ModelT &model,
    const rmagine::Transform &Tsb,
    const rmagine::Transform &Tbm,
    const gz::msgs::Time &stamp,
    const gz::math::Pose3d &base_pose,
    const gz::math::Pose3d &sensor_pose);

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity frame_entity_{gz::sim::kNullEntity};

  std::string map_key_{"default"};
  std::string frame_id_{"sensor"};
  std::string topic_scan_{"scan"};
  std::string topic_points_{"points"};
  double update_rate_{10.0};
  bool debug_{false};

  // Ports Classic's per-sensor multi-output fan-out (an arbitrary list of
  // (topic, message type) pairs from one sensor). `topic_scan_`/
  // `topic_points_` above still name the *first*/default entry in each
  // list; additional `<output><topic>...</topic><type>scan|points</type>
  // </output>` SDF elements append more publishers of the same underlying
  // message types.
  std::vector<std::string> extra_scan_topics_;
  std::vector<std::string> extra_points_topics_;

  rmagine::EmbreeMapPtr map_;
  // Fetched from MapRegistry alongside `map_` (same key) -- the map system
  // holds the SAME mutex object and takes a unique_lock around its whole
  // incremental-sync block, so a shared_lock here is required around every
  // simulate() call now that the map is mutated in place across several
  // commit()s per tick instead of swapped atomically. Null until the map
  // system has registered one (i.e. until RefreshSimulator finds a map).
  std::shared_ptr<std::shared_mutex> map_mutex_;
  SensorModelConfig model_cfg_;
  // Exactly one of these is ever constructed, matching model_cfg_.type --
  // see the class-level comment on RunAndPublish.
  rmagine::SphereSimulatorEmbreePtr sim_spherical_;
  rmagine::PinholeSimulatorEmbreePtr sim_pinhole_;
  rmagine::O1DnSimulatorEmbreePtr sim_o1dn_;
  rmagine::OnDnSimulatorEmbreePtr sim_ondn_;
  uint64_t map_revision_{0};
  gz::math::Pose3d local_sensor_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool frame_resolved_logged_{false};

  // Non-owning -- points at RmagineEmbreeSensorSystem's shared gz-transport
  // node, set once in Load() and valid for this instance's whole lifetime
  // (an instance never outlives its owning factory system).
  gz::transport::Node *gz_node_{nullptr};
  // scan_pubs_[0]/points_pubs_[0] are always the topic_scan_/topic_points_
  // default; any parsed <output> entries follow.
  std::vector<gz::transport::Node::Publisher> scan_pubs_;
  std::vector<gz::transport::Node::Publisher> points_pubs_;

  std::chrono::nanoseconds last_pub_time_{0};
  bool has_published_{false};
};

// Attached once per world (like RmagineEmbreeMapSystem), NOT nested inside
// each <sensor> -- discovers every `<sensor type="custom"
// gz:type="rmagine_embree">` anywhere in the world via
// gz::sim::components::CustomSensor and owns one RmagineEmbreeSensorInstance
// per discovered entity. This is the closest available analogue to Gazebo
// Classic's GZ_REGISTER_STATIC_SENSOR-registered sensor types: gz-sensors'
// own plugin-loading mechanism for custom Sensor subclasses was removed
// upstream (confirmed in the installed gz-sensors8 SensorFactory.hh), but
// gz-sim's CustomSensor component exists specifically so a generic System
// can fill that role instead -- see gz-sim's own
// share/gz/gz-sim8/worlds/environmental_sensor.sdf for the same pattern
// (`type="custom" gz:type="..."`).
class RmagineEmbreeSensorSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RmagineEmbreeSensorSystem() = default;
  ~RmagineEmbreeSensorSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::transport::Node gz_node_;
  std::unordered_map<gz::sim::Entity, std::unique_ptr<RmagineEmbreeSensorInstance>> instances_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_EMBREE_SENSOR_SYSTEM_HPP
