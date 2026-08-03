#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <gz/math/Pose3.hh>

#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>

namespace rmagine_gazebo_plugins
{

// Shared by both rmagine_embree_sensor_system and rmagine_optix_sensor_system
// -- identical logic in both, they only differ in how `ranges` (always a
// plain RAM float buffer by the time this is called) was produced. Templated
// on the model type so it works unchanged for Spherical/Pinhole/O1Dn/OnDn:
// all four expose the same getWidth()/getHeight()/getBufferId()/
// getDirection()/range interface (see rmagine/types/sensor_models.h).
template<typename ModelT, typename RangesT>
void LogSimulationSummary(
  bool debug,
  const std::string &system_name,
  const std::string &frame_id,
  const gz::math::Pose3d &base_pose,
  const gz::math::Pose3d &sensor_pose,
  const gz::math::Pose3d &local_sensor_pose,
  const ModelT &model,
  const RangesT &ranges)
{
  if(!debug)
  {
    return;
  }

  auto pose_summary = [](const gz::math::Pose3d &pose)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "pos=(" << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Pos().Z() << ") "
        << "rpy=(" << pose.Rot().Roll() << "," << pose.Rot().Pitch() << "," << pose.Rot().Yaw() << ")";
    return out.str();
  };

  size_t finite_ranges = 0;
  float min_range = std::numeric_limits<float>::max();
  float max_range = 0.0f;
  for(size_t i = 0; i < ranges.size(); ++i)
  {
    const float range = ranges[i];
    if(std::isfinite(range) && model.range.inside(range))
    {
      finite_ranges++;
      min_range = std::min(min_range, range);
      max_range = std::max(max_range, range);
    }
  }

  std::cerr << "[" << system_name << "] frame='" << frame_id
            << "' base{" << pose_summary(base_pose) << "} "
            << "sensor{" << pose_summary(sensor_pose) << "} "
            << "local{" << pose_summary(local_sensor_pose) << "} "
            << "finite_ranges=" << finite_ranges << "/" << ranges.size();

  if(finite_ranges > 0)
  {
    std::cerr << " min=" << std::fixed << std::setprecision(3) << min_range
              << " max=" << max_range;
  }
  else
  {
    std::cerr << " min=n/a max=n/a";
  }
  std::cerr << std::endl;
}

// LaserScan only makes sense for a single-row spherical scan (a rotating
// 2D lidar) -- Pinhole/O1Dn/OnDn have no equivalent flat representation,
// so this is a no-op for them (checked at compile time, not just runtime,
// so it never even touches model.phi/.theta on models that don't have
// those members).
template<typename ModelT, typename RangesT>
void PublishLaserScanIfApplicable(
  const ModelT &model,
  const RangesT &ranges,
  const rclcpp::Time &stamp,
  const std::string &frame_id,
  double update_rate,
  const std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> &scan_pubs)
{
  if constexpr (std::is_same_v<ModelT, rmagine::SphericalModel>)
  {
    if(model.phi.size != 1 || scan_pubs.empty())
    {
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame_id;
    scan.range_min = model.range.min;
    scan.range_max = model.range.max;
    scan.angle_min = model.theta.min;
    scan.angle_increment = model.theta.inc;
    if(model.theta.size > 0)
    {
      scan.angle_max = model.theta.min
        + model.theta.inc * static_cast<float>(model.theta.size - 1);
    } else {
      scan.angle_max = model.theta.min;
    }
    if(update_rate > 0.0)
    {
      scan.scan_time = static_cast<float>(1.0 / update_rate);
    }
    if(model.theta.size > 1 && update_rate > 0.0)
    {
      scan.time_increment = scan.scan_time / static_cast<float>(model.theta.size - 1);
    }
    scan.ranges.resize(ranges.size());
    for(size_t i = 0; i < ranges.size(); ++i)
    {
      scan.ranges[i] = ranges[i];
    }
    for(const auto &scan_pub : scan_pubs)
    {
      if(scan_pub)
      {
        scan_pub->publish(scan);
      }
    }
  }
}

// Optional extra per-point data a simulate() call can produce alongside
// Ranges (see rmagine/simulation/SimulationResults.hpp) -- Classic's ROS 1
// PointCloud2 publisher could emit `ring`/normals/`obj_id`/`face_id` on
// top of x/y/z; the Harmonic port only ever requested Ranges, so none of
// this was even computed, not just dropped at publish time. nullptr means
// "not requested/available", and that field is omitted from the message
// entirely (not zero-filled) -- callers only pay for what they ask
// `RunAndPublish`'s ResT Bundle to compute.
struct PointCloudExtras
{
  const rmagine::Vector* normals = nullptr;
  const unsigned int* object_ids = nullptr;
  const unsigned int* face_ids = nullptr;
};

template<typename ModelT, typename RangesT>
void PublishPointCloud(
  const ModelT &model,
  const RangesT &ranges,
  const rclcpp::Time &stamp,
  const std::string &frame_id,
  const std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> &points_pubs,
  const PointCloudExtras &extras = PointCloudExtras{})
{
  if(points_pubs.empty())
  {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = model.getHeight();
  msg.width = model.getWidth();
  msg.is_dense = false;

  auto add_field = [&msg](const std::string &name, uint8_t datatype, uint32_t size) -> uint32_t
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = msg.point_step;
    field.datatype = datatype;
    field.count = 1;
    msg.fields.push_back(field);
    const uint32_t offset = msg.point_step;
    msg.point_step += size;
    return offset;
  };

  msg.fields.resize(0);
  msg.point_step = 0;
  const uint32_t off_x = add_field("x", sensor_msgs::msg::PointField::FLOAT32, sizeof(float));
  add_field("y", sensor_msgs::msg::PointField::FLOAT32, sizeof(float));
  add_field("z", sensor_msgs::msg::PointField::FLOAT32, sizeof(float));
  // ring: which row (vid) a point came from -- a rotating-scanner concept
  // (Spherical's phi rows), but well-defined identically for every model
  // type here (just the row index), so always published, not gated behind
  // model type the way LaserScan is.
  const uint32_t off_ring = add_field("ring", sensor_msgs::msg::PointField::UINT16, sizeof(uint16_t));
  const uint32_t off_nx = extras.normals
    ? add_field("normal_x", sensor_msgs::msg::PointField::FLOAT32, sizeof(float)) : 0;
  if(extras.normals)
  {
    add_field("normal_y", sensor_msgs::msg::PointField::FLOAT32, sizeof(float));
    add_field("normal_z", sensor_msgs::msg::PointField::FLOAT32, sizeof(float));
  }
  const uint32_t off_obj_id = extras.object_ids
    ? add_field("obj_id", sensor_msgs::msg::PointField::UINT32, sizeof(uint32_t)) : 0;
  const uint32_t off_face_id = extras.face_ids
    ? add_field("face_id", sensor_msgs::msg::PointField::UINT32, sizeof(uint32_t)) : 0;

  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.width * msg.height * msg.point_step);

  for(size_t vid = 0; vid < model.getHeight(); vid++)
  {
    for(size_t hid = 0; hid < model.getWidth(); hid++)
    {
      const unsigned int pid = model.getBufferId(vid, hid);
      const float range = ranges[pid];
      uint8_t* buff = &msg.data[pid * msg.point_step];
      rmagine::Vector3* p = reinterpret_cast<rmagine::Vector3*>(buff + off_x);
      if(model.range.inside(range))
      {
        // getOrigin() is {0,0,0} for Spherical/Pinhole (a single shared
        // origin at the sensor frame's own origin) but non-zero for
        // O1Dn/OnDn, whose rays can originate away from the sensor
        // frame's origin -- must be added back in, or points silently
        // collapse onto the wrong ray.
        *p = model.getOrigin(vid, hid) + model.getDirection(vid, hid) * range;
      } else {
        *p = rmagine::Vector3::NaN();
      }

      *reinterpret_cast<uint16_t*>(buff + off_ring) = static_cast<uint16_t>(vid);

      if(extras.normals)
      {
        rmagine::Vector3* n = reinterpret_cast<rmagine::Vector3*>(buff + off_nx);
        *n = extras.normals[pid];
      }
      if(extras.object_ids)
      {
        *reinterpret_cast<uint32_t*>(buff + off_obj_id) = extras.object_ids[pid];
      }
      if(extras.face_ids)
      {
        *reinterpret_cast<uint32_t*>(buff + off_face_id) = extras.face_ids[pid];
      }
    }
  }

  for(const auto &points_pub : points_pubs)
  {
    if(points_pub)
    {
      points_pub->publish(msg);
    }
  }
}

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP
