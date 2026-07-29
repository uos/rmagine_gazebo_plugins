#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP

#include <memory>
#include <string>
#include <cstdint>
#include <limits>

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>
#include <rmagine/simulation/O1DnSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/noise/NoiseCuda.hpp>
#include <vector>

#include "rmagine_gazebo_plugins/gz/sensor_model_config.hpp"

namespace rmagine_gazebo_plugins
{

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
  void LoadParams(const std::shared_ptr<const sdf::Element> &_sdf);
  void RefreshSimulator();
  void ResolveFrameEntity(const gz::sim::EntityComponentManager &_ecm);

  // Runs setTsb/setModel/simulate for whichever (simulator, model) pair is
  // currently active, then publishes PointCloud2 (and LaserScan, for
  // Spherical) via the shared templated helpers in sensor_model_publish.hpp.
  template<typename SimPtrT, typename ModelT>
  void RunAndPublish(
    SimPtrT &sim,
    const ModelT &model,
    const rmagine::Transform &Tsb,
    const rmagine::Transform &Tbm,
    const rclcpp::Time &stamp,
    const gz::math::Pose3d &base_pose,
    const gz::math::Pose3d &sensor_pose);

  gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
  gz::sim::Entity frame_entity_{gz::sim::kNullEntity};

  std::string map_key_{"default"};
  std::string parent_frame_id_{"world"};
  std::string frame_id_{"sensor"};
  std::string topic_scan_{"scan"};
  std::string topic_points_{"points"};
  double update_rate_{10.0};
  bool debug_{false};

  // Ports Classic's per-sensor multi-output fan-out -- see the identical
  // comment on rmagine_embree_sensor_system.hpp's own
  // extra_scan_topics_/extra_points_topics_.
  std::vector<std::string> extra_scan_topics_;
  std::vector<std::string> extra_points_topics_;

  rmagine::OptixMapPtr map_;
  SensorModelConfig model_cfg_;
  // Exactly one of these is ever constructed, matching model_cfg_.type --
  // see the class-level comment on RunAndPublish.
  rmagine::SphereSimulatorOptixPtr sim_spherical_;
  rmagine::PinholeSimulatorOptixPtr sim_pinhole_;
  rmagine::O1DnSimulatorOptixPtr sim_o1dn_;
  rmagine::OnDnSimulatorOptixPtr sim_ondn_;
  uint64_t map_revision_{0};
  gz::math::Pose3d local_sensor_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool frame_resolved_logged_{false};

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // scan_pubs_[0]/points_pubs_[0] are always the topic_scan_/topic_points_
  // default; any parsed <output> entries follow.
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scan_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> points_pubs_;

  // Applied to simulated ranges (in VRAM, before download to RAM) in the
  // order parsed from SDF -- ports Classic's OptiX-only noise support
  // (rmagine_optix_spherical_gzplugin.cpp), never carried over to the
  // Harmonic System. See LoadParams() for the SDF schema.
  std::vector<rmagine::NoiseCudaPtr> noise_models_;

  std::chrono::nanoseconds last_pub_time_{0};
  bool has_published_{false};
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_RMAGINE_OPTIX_SENSOR_SYSTEM_HPP
