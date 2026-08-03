#include "rmagine_gazebo_plugins/gz/rmagine_optix_sensor_system.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/CustomSensor.hh>

#include <sdf/Sensor.hh>

#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/noise/GaussianNoiseCuda.hpp>
#include <rmagine/noise/UniformDustNoiseCuda.hpp>
#include <rmagine/noise/RelGaussianNoiseCuda.hpp>
#include <iostream>
#include <cmath>

#include "rmagine_gazebo_plugins/gz/optix_map_registry.hpp"
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

static gz::msgs::Time ToGzTime(std::chrono::nanoseconds t)
{
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(t);
  gz::msgs::Time time;
  time.set_sec(static_cast<int64_t>(sec.count()));
  time.set_nsec(static_cast<int32_t>((t - sec).count()));
  return time;
}

template<typename SimPtrT, typename ModelT>
void RmagineOptixSensorInstance::RunAndPublish(
  SimPtrT &sim,
  const ModelT &model,
  const rmagine::Transform &Tsb,
  const rmagine::Transform &Tbm,
  const gz::msgs::Time &stamp,
  const gz::math::Pose3d &base_pose,
  const gz::math::Pose3d &sensor_pose)
{
  if(!sim)
  {
    sim = std::make_shared<typename SimPtrT::element_type>(map_);
  }

  sim->setTsb(Tsb);
  sim->setModel(model);

  using ResT = rmagine::Bundle<
    rmagine::Ranges<rmagine::VRAM_CUDA>,
    rmagine::Normals<rmagine::VRAM_CUDA>,
    rmagine::ObjectIds<rmagine::VRAM_CUDA>,
    rmagine::FaceIds<rmagine::VRAM_CUDA>>;
  ResT res_gpu;
  rmagine::resize_memory_bundle<rmagine::VRAM_CUDA>(
    res_gpu, model.getWidth(), model.getHeight(), 1);
  {
    // See the identical comment in rmagine_embree_sensor_system.cpp's
    // RunAndPublish -- same rationale, GPU side.
    std::shared_lock<std::shared_mutex> lock;
    if(map_mutex_)
    {
      lock = std::shared_lock<std::shared_mutex>(*map_mutex_);
    }
    sim->template simulate<ResT>(Tbm, res_gpu);
  }

  // Applied in VRAM, before download.
  for(auto &noise_model : noise_models_)
  {
    noise_model->apply(res_gpu.ranges);
  }

  rmagine::Memory<float, rmagine::RAM> ranges = res_gpu.ranges;
  rmagine::Memory<rmagine::Vector, rmagine::RAM> normals = res_gpu.normals;
  rmagine::Memory<unsigned int, rmagine::RAM> object_ids = res_gpu.object_ids;
  rmagine::Memory<unsigned int, rmagine::RAM> face_ids = res_gpu.face_ids;

  LogSimulationSummary(debug_, "RmagineOptixSensorInstance", frame_id_,
    base_pose, sensor_pose, local_sensor_pose_, model, ranges);

  PublishLaserScanIfApplicable(model, ranges, stamp, frame_id_, scan_pubs_);

  PointCloudExtras extras;
  extras.normals = normals.raw();
  extras.object_ids = object_ids.raw();
  extras.face_ids = face_ids.raw();
  PublishPointCloud(model, ranges, stamp, frame_id_, points_pubs_, extras);
}

void RmagineOptixSensorInstance::RefreshSimulator()
{
  auto &registry = OptixMapRegistry::Instance();
  const auto revision = registry.GetOptixMapRevision(map_key_);
  const auto map = registry.GetOptixMap(map_key_);
  if(!map)
  {
    map_.reset();
    map_mutex_.reset();
    sim_spherical_.reset();
    sim_pinhole_.reset();
    sim_o1dn_.reset();
    sim_ondn_.reset();
    map_revision_ = 0;
    std::cerr << "[RmagineOptixSensorInstance] No map available for key '" << map_key_ << "'." << std::endl;
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
    std::cerr << "[RmagineOptixSensorInstance] Refreshed simulator for map key '"
              << map_key_ << "' at revision " << map_revision_ << "." << std::endl;
  }
}

void RmagineOptixSensorInstance::ResolveFrameEntity(
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
          std::cerr << "[RmagineOptixSensorInstance] Resolved frame entity '" << frame_id_
                    << "' to entity " << frame_entity_ << "." << std::endl;
          frame_resolved_logged_ = true;
        }
        return false;
      }

      return true;
    });
}

void RmagineOptixSensorInstance::Load(
  gz::sim::Entity sensor_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::transport::Node *gz_node)
{
  sensor_entity_ = sensor_entity;
  gz_node_ = gz_node;

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

    // Ports Classic's OptiX-only noise support -- deliberately flat
    // (repeated <noise> elements directly under the sensor, not nested
    // under a <ray> wrapper like Classic did), matching every other SDF
    // element here.
    if(_sdf->HasElement("noise"))
    {
      auto sdf_mut = const_cast<sdf::Element*>(_sdf.get());
      auto noiseElem = sdf_mut->GetElement("noise");
      while(noiseElem)
      {
        const std::string noise_type = noiseElem->Get<std::string>("type");
        if(noise_type == "gaussian")
        {
          float mean = 0.0f;
          if(noiseElem->HasElement("mean"))
          {
            mean = noiseElem->Get<float>("mean");
          }
          const float stddev = noiseElem->Get<float>("stddev");
          noise_models_.push_back(std::make_shared<rmagine::GaussianNoiseCuda>(mean, stddev));
        } else if(noise_type == "uniform_dust") {
          const float hit_prob = noiseElem->Get<float>("hit_prob");
          const float return_prob = noiseElem->Get<float>("return_prob");
          noise_models_.push_back(
            std::make_shared<rmagine::UniformDustNoiseCuda>(hit_prob, return_prob));
        } else if(noise_type == "rel_gaussian") {
          float mean = 0.0f;
          if(noiseElem->HasElement("mean"))
          {
            mean = noiseElem->Get<float>("mean");
          }
          const float stddev = noiseElem->Get<float>("stddev");
          float range_exp = 1.0f;
          if(noiseElem->HasElement("range_exp"))
          {
            range_exp = noiseElem->Get<float>("range_exp");
          }
          noise_models_.push_back(
            std::make_shared<rmagine::RelGaussianNoiseCuda>(mean, stddev, range_exp));
        } else {
          std::cerr << "[RmagineOptixSensorInstance] Unknown noise type '" << noise_type
                    << "' -- skipping." << std::endl;
        }
        noiseElem = noiseElem->GetNextElement("noise");
      }
    }

    // Ports Classic's per-sensor multi-output fan-out.
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
            std::cerr << "[RmagineOptixSensorInstance] Unknown output type '" << type
                      << "' for topic '" << topic << "' -- skipping." << std::endl;
          }
        }
        outputElem = outputElem->GetNextElement("output");
      }
    }
  }

  scan_pubs_.push_back(gz_node_->Advertise<gz::msgs::LaserScan>(topic_scan_));
  for(const auto &topic : extra_scan_topics_)
  {
    scan_pubs_.push_back(gz_node_->Advertise<gz::msgs::LaserScan>(topic));
  }
  points_pubs_.push_back(gz_node_->Advertise<gz::msgs::PointCloudPacked>(topic_points_));
  for(const auto &topic : extra_points_topics_)
  {
    points_pubs_.push_back(gz_node_->Advertise<gz::msgs::PointCloudPacked>(topic));
  }

  RefreshSimulator();
}

void RmagineOptixSensorInstance::Update(
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
  const gz::msgs::Time stamp = ToGzTime(sim_now);

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
// See the identical comment in rmagine_embree_sensor_system.cpp.
constexpr const char *kGzTypeRmagineOptix = "rmagine_optix";
}  // namespace

void RmagineOptixSensorSystem::Configure(
  const gz::sim::Entity &,
  const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &,
  gz::sim::EventManager &)
{
  // gz_node_ default-constructs to a valid, usable gz-transport node --
  // no ROS/rclcpp init needed anywhere in this plugin anymore.
}

void RmagineOptixSensorSystem::PostUpdate(
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
      if(gz_type != kGzTypeRmagineOptix)
      {
        return true;
      }

      auto instance = std::make_unique<RmagineOptixSensorInstance>();
      instance->Load(entity, elem, &gz_node_);
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

GZ_ADD_PLUGIN(rmagine_gazebo_plugins::RmagineOptixSensorSystem,
              gz::sim::System,
              rmagine_gazebo_plugins::RmagineOptixSensorSystem::ISystemConfigure,
              rmagine_gazebo_plugins::RmagineOptixSensorSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rmagine_gazebo_plugins::RmagineOptixSensorSystem,
                    "rmagine_optix_sensor_system")
