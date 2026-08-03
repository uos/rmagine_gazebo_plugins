#include "rmagine_gazebo_plugins/gz/rmagine_embree_sensor_system.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/CustomSensor.hh>

#include <sdf/Sensor.hh>

#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Memory.hpp>
#include <iostream>
#include <cmath>

#include "rmagine_gazebo_plugins/gz/map_registry.hpp"
#include "rmagine_gazebo_plugins/gz/sensor_model_publish.hpp"

namespace rmagine_gazebo_plugins
{

static rmagine::Transform ToRmTransform(const gz::math::Pose3d &pose)
{
  rmagine::Transform T;
  T.R.x = pose.Rot().X();
  T.R.y = pose.Rot().Y();
  T.R.z = pose.Rot().Z();
  T.R.w = pose.Rot().W();
  T.t.x = pose.Pos().X();
  T.t.y = pose.Pos().Y();
  T.t.z = pose.Pos().Z();
  return T;
}

template<typename SimPtrT, typename ModelT>
void RmagineEmbreeSensorInstance::RunAndPublish(
  SimPtrT &sim,
  const ModelT &model,
  const rmagine::Transform &Tsb,
  const rmagine::Transform &Tbm,
  const rclcpp::Time &stamp,
  const gz::math::Pose3d &base_pose,
  const gz::math::Pose3d &sensor_pose)
{
  if(!sim)
  {
    sim = std::make_shared<typename SimPtrT::element_type>(map_);
  }

  sim->setTsb(Tsb);
  sim->setModel(model);

  // Ranges alone matches what was published before -- Normals/ObjectIds/
  // FaceIds are also requested now so PointCloud2 can carry the same
  // per-point fields Classic's ROS 1 publisher did (see
  // sensor_model_publish.hpp's PointCloudExtras). Embree computes these
  // as part of the same intersection query, not a second pass, so this
  // is cheap on the CPU/Embree path.
  using ResT = rmagine::Bundle<
    rmagine::Ranges<rmagine::RAM>,
    rmagine::Normals<rmagine::RAM>,
    rmagine::ObjectIds<rmagine::RAM>,
    rmagine::FaceIds<rmagine::RAM>>;
  ResT res;
  {
    // The map system now mutates its persistent EmbreeScene in place
    // (add/remove/move geometry, then commit()) across several steps of
    // its own PostUpdate, instead of one atomic pointer swap -- a
    // shared_lock here (paired with the map system's unique_lock around
    // its whole sync block, see rmagine_embree_map_system.cpp) is required
    // so a raycast never runs concurrently with a scene mutation.
    std::shared_lock<std::shared_mutex> lock;
    if(map_mutex_)
    {
      lock = std::shared_lock<std::shared_mutex>(*map_mutex_);
    }
    res = sim->template simulate<ResT>(Tbm);
  }
  auto &ranges = res.ranges;

  LogSimulationSummary(debug_, "RmagineEmbreeSensorInstance", frame_id_,
    base_pose, sensor_pose, local_sensor_pose_, model, ranges);

  PublishLaserScanIfApplicable(model, ranges, stamp, frame_id_, update_rate_, scan_pubs_);

  PointCloudExtras extras;
  extras.normals = res.normals.raw();
  extras.object_ids = res.object_ids.raw();
  extras.face_ids = res.face_ids.raw();
  PublishPointCloud(model, ranges, stamp, frame_id_, points_pubs_, extras);
}

void RmagineEmbreeSensorInstance::RefreshSimulator()
{
  auto &registry = MapRegistry::Instance();
  const auto revision = registry.GetEmbreeMapRevision(map_key_);
  const auto map = registry.GetEmbreeMap(map_key_);
  if(!map)
  {
    map_.reset();
    map_mutex_.reset();
    sim_spherical_.reset();
    sim_pinhole_.reset();
    sim_o1dn_.reset();
    sim_ondn_.reset();
    map_revision_ = 0;
    std::cerr << "[RmagineEmbreeSensorInstance] No map available for key '" << map_key_ << "'." << std::endl;
    return;
  }

  if(!map_ || map != map_ || revision != map_revision_)
  {
    map_ = map;
    map_mutex_ = registry.GetMapMutex(map_key_);
    sim_spherical_.reset();
    sim_pinhole_.reset();
    sim_o1dn_.reset();
    sim_ondn_.reset();
    map_revision_ = revision;
    std::cerr << "[RmagineEmbreeSensorInstance] Refreshed simulator for map key '"
              << map_key_ << "' at revision " << map_revision_ << "." << std::endl;
  }
}

void RmagineEmbreeSensorInstance::ResolveFrameEntity(
  const gz::sim::EntityComponentManager &_ecm)
{
  if(frame_entity_ != gz::sim::kNullEntity)
  {
    return;
  }

  _ecm.Each<gz::sim::components::Name, gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::Name *nameComp,
        const gz::sim::components::ParentEntity *parentComp) -> bool
    {
      if(!nameComp || !parentComp)
      {
        return true;
      }

      if(nameComp->Data() == frame_id_ && parentComp->Data() == sensor_entity_)
      {
        frame_entity_ = entity;
        if(debug_ && !frame_resolved_logged_)
        {
          std::cerr << "[RmagineEmbreeSensorInstance] Resolved frame entity '" << frame_id_
                    << "' to entity " << frame_entity_ << "." << std::endl;
          frame_resolved_logged_ = true;
        }
        return false;
      }

      return true;
    });
}

void RmagineEmbreeSensorInstance::Load(
  gz::sim::Entity sensor_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  const rclcpp::Node::SharedPtr &node,
  tf2_ros::TransformBroadcaster *tf_broadcaster)
{
  sensor_entity_ = sensor_entity;
  node_ = node;
  tf_broadcaster_ = tf_broadcaster;

  model_cfg_ = LoadSensorModelConfig(_sdf);

  if(_sdf)
  {
    if(_sdf->HasElement("map_key"))
    {
      map_key_ = _sdf->Get<std::string>("map_key");
    }
    if(_sdf->HasElement("frame"))
    {
      frame_id_ = _sdf->Get<std::string>("frame");
    }
    if(_sdf->HasElement("parent_frame"))
    {
      parent_frame_id_ = _sdf->Get<std::string>("parent_frame");
    }
    if(_sdf->HasElement("topic_scan"))
    {
      topic_scan_ = _sdf->Get<std::string>("topic_scan");
    }
    if(_sdf->HasElement("topic_points"))
    {
      topic_points_ = _sdf->Get<std::string>("topic_points");
    }
    if(_sdf->HasElement("update_rate"))
    {
      update_rate_ = _sdf->Get<double>("update_rate");
    }
    if(_sdf->HasElement("debug"))
    {
      debug_ = _sdf->Get<bool>("debug");
    }

    // Ports Classic's per-sensor multi-output fan-out -- see the header's
    // comment on extra_scan_topics_/extra_points_topics_. Each <output>
    // must have a <topic> and a <type> of "scan" or "points"; anything else
    // is skipped with a warning.
    if(_sdf->HasElement("output"))
    {
      auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
      auto outputElem = sdf_mut->GetElement("output");
      while(outputElem)
      {
        if(outputElem->HasElement("topic") && outputElem->HasElement("type"))
        {
          const std::string topic = outputElem->Get<std::string>("topic");
          const std::string type = outputElem->Get<std::string>("type");
          if(type == "scan")
          {
            extra_scan_topics_.push_back(topic);
          } else if(type == "points") {
            extra_points_topics_.push_back(topic);
          } else {
            std::cerr << "[RmagineEmbreeSensorInstance] Unknown output type '" << type
                      << "' for topic '" << topic << "' -- skipping." << std::endl;
          }
        }
        outputElem = outputElem->GetNextElement("output");
      }
    }
  }

  // scan_pubs_[0]/points_pubs_[0] are always the topic_scan_/topic_points_
  // default; parsed <output> entries follow -- see the header's comment on
  // those members.
  scan_pubs_.push_back(node_->create_publisher<sensor_msgs::msg::LaserScan>(topic_scan_, 1));
  for(const auto &topic : extra_scan_topics_)
  {
    scan_pubs_.push_back(node_->create_publisher<sensor_msgs::msg::LaserScan>(topic, 1));
  }
  points_pubs_.push_back(node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_points_, 1));
  for(const auto &topic : extra_points_topics_)
  {
    points_pubs_.push_back(node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1));
  }

  RefreshSimulator();
}

void RmagineEmbreeSensorInstance::Update(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if(_info.paused)
  {
    return;
  }

  ResolveFrameEntity(_ecm);

  const auto sim_now = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
  const auto base_pose = gz::sim::worldPose(sensor_entity_, _ecm);
  gz::math::Pose3d sensor_pose = base_pose;
  if(frame_entity_ != gz::sim::kNullEntity)
  {
    sensor_pose = gz::sim::worldPose(frame_entity_, _ecm);
    local_sensor_pose_ = base_pose.Inverse() * sensor_pose;
  }
  const rclcpp::Time stamp(sim_now.count(), RCL_ROS_TIME);

  if(tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = parent_frame_id_;
    tf_msg.child_frame_id = frame_id_;
    tf_msg.transform.translation.x = sensor_pose.Pos().X();
    tf_msg.transform.translation.y = sensor_pose.Pos().Y();
    tf_msg.transform.translation.z = sensor_pose.Pos().Z();
    tf_msg.transform.rotation.x = sensor_pose.Rot().X();
    tf_msg.transform.rotation.y = sensor_pose.Rot().Y();
    tf_msg.transform.rotation.z = sensor_pose.Rot().Z();
    tf_msg.transform.rotation.w = sensor_pose.Rot().W();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));
  if(has_published_ && (sim_now - last_pub_time_) < period)
  {
    return;
  }

  RefreshSimulator();

  if(!map_)
  {
    return;
  }

  const rmagine::Transform Tsb = ToRmTransform(local_sensor_pose_);
  const rmagine::Transform Tbm = ToRmTransform(base_pose);

  switch(model_cfg_.type)
  {
    case SensorModelType::Spherical:
      RunAndPublish(sim_spherical_, model_cfg_.spherical, Tsb, Tbm, stamp, base_pose, sensor_pose);
      break;
    case SensorModelType::Pinhole:
      RunAndPublish(sim_pinhole_, model_cfg_.pinhole, Tsb, Tbm, stamp, base_pose, sensor_pose);
      break;
    case SensorModelType::O1Dn:
      RunAndPublish(sim_o1dn_, model_cfg_.o1dn, Tsb, Tbm, stamp, base_pose, sensor_pose);
      break;
    case SensorModelType::OnDn:
      RunAndPublish(sim_ondn_, model_cfg_.ondn, Tsb, Tbm, stamp, base_pose, sensor_pose);
      break;
  }

  last_pub_time_ = sim_now;
  has_published_ = true;
}

namespace
{
// gz-sim's own convention for third-party sensor types (confirmed from its
// shipped share/gz/gz-sim8/worlds/environmental_sensor.sdf example):
// `<sensor type="custom" gz:type="...">`. `type="custom"` is required --
// sdformat validates the standard `type` attribute against sdf::SensorType's
// known names, so an arbitrary string there would be rejected; `gz:type` is
// a free-form, schema-unvalidated escape hatch read like any other
// attribute/child via sdf::Element::Get.
constexpr const char *kGzTypeRmagineEmbree = "rmagine_embree";
}  // namespace

void RmagineEmbreeSensorSystem::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  if(!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  std::string node_name = "rmagine_embree_sensor_system";
  if(_sdf && _sdf->HasElement("node_name"))
  {
    node_name = _sdf->Get<std::string>("node_name");
  }
  node_ = std::make_shared<rclcpp::Node>(node_name);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void RmagineEmbreeSensorSystem::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::CustomSensor *sensorComp,
        const gz::sim::components::ParentEntity *) -> bool
    {
      const sdf::Sensor &sensorData = sensorComp->Data();
      if(sensorData.TypeStr() != "custom")
      {
        return true;
      }

      auto elem = sensorData.Element();
      const std::string gz_type = elem
        ? elem->Get<std::string>("gz:type", std::string("")).first
        : std::string("");
      if(gz_type != kGzTypeRmagineEmbree)
      {
        return true;
      }

      auto instance = std::make_unique<RmagineEmbreeSensorInstance>();
      instance->Load(entity, elem, node_, tf_broadcaster_.get());
      instances_[entity] = std::move(instance);
      return true;
    });

  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
    [&](const gz::sim::Entity &entity,
        const gz::sim::components::CustomSensor *) -> bool
    {
      instances_.erase(entity);
      return true;
    });

  for(auto &kv : instances_)
  {
    kv.second->Update(_info, _ecm);
  }
}

}  // namespace rmagine_gazebo_plugins

GZ_ADD_PLUGIN(rmagine_gazebo_plugins::RmagineEmbreeSensorSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::RmagineEmbreeSensorSystem::ISystemConfigure,
              rmagine_gazebo_plugins::RmagineEmbreeSensorSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::RmagineEmbreeSensorSystem,
                    "rmagine_embree_sensor_system")
