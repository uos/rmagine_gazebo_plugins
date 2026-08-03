#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP

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

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>
#include <rmagine/simulation/O1DnSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/noise/NoiseCuda.hpp>

#include "rmagine_gazebo_plugins/gz/sensor_model_config.hpp"

namespace rmagine_gazebo_plugins
{

// GPU mirror of RmagineEmbreeSensorInstance -- see its header comment
// (gz-transport-native publishing, no TF -- gz::sim::systems::PosePublisher
// + ros_gz_bridge covers it).
class RmagineOptixSensorInstance
{
public:
  void Load(
    gz::sim::Entity sensor_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::transport::Node *gz_node);

  void Update(const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm);

private:
  void RefreshSimulator();
  void ResolveFrameEntity(const gz::sim::EntityComponentManager &_ecm);

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

  std::vector<std::string> extra_scan_topics_;
  std::vector<std::string> extra_points_topics_;

  rmagine::OptixMapPtr map_;
  std::shared_ptr<std::shared_mutex> map_mutex_;
  SensorModelConfig model_cfg_;
  rmagine::SphereSimulatorOptixPtr sim_spherical_;
  rmagine::PinholeSimulatorOptixPtr sim_pinhole_;
  rmagine::O1DnSimulatorOptixPtr sim_o1dn_;
  rmagine::OnDnSimulatorOptixPtr sim_ondn_;
  uint64_t map_revision_{0};
  gz::math::Pose3d local_sensor_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool frame_resolved_logged_{false};

  // Non-owning -- see the identical comment on RmagineEmbreeSensorInstance's
  // own gz_node_.
  gz::transport::Node *gz_node_{nullptr};
  std::vector<gz::transport::Node::Publisher> scan_pubs_;
  std::vector<gz::transport::Node::Publisher> points_pubs_;

  // Applied to simulated ranges (in VRAM, before download to RAM) in the
  // order parsed from SDF.
  std::vector<rmagine::NoiseCudaPtr> noise_models_;

  std::chrono::nanoseconds last_pub_time_{0};
  bool has_published_{false};
};

// GPU mirror of RmagineEmbreeSensorSystem -- see its header comment for the
// gz-sim CustomSensor discovery rationale. Attached once per world;
// discovers `<sensor type="custom" gz:type="rmagine_optix">`.
class RmagineOptixSensorSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  RmagineOptixSensorSystem() = default;
  ~RmagineOptixSensorSystem() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::transport::Node gz_node_;
  std::unordered_map<gz::sim::Entity, std::unique_ptr<RmagineOptixSensorInstance>> instances_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP
