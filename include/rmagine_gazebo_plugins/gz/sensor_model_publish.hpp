#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/time.pb.h>
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
// those members). gz::msgs::LaserScan actually supports a vertical axis
// too (unlike sensor_msgs/LaserScan), but this stays gated to phi.size==1
// since that's the only case ros_gz_bridge's LaserScan conversion (and
// ROS's own LaserScan message) can represent -- a multi-ring scan should
// use PublishPointCloud instead, same as before.
template<typename ModelT, typename RangesT>
void PublishLaserScanIfApplicable(
  const ModelT &model,
  const RangesT &ranges,
  const gz::msgs::Time &stamp,
  const std::string &frame_id,
  std::vector<gz::transport::Node::Publisher> &scan_pubs)
{
  if constexpr (std::is_same_v<ModelT, rmagine::SphericalModel>)
  {
    if(model.phi.size != 1 || scan_pubs.empty())
    {
      return;
    }

    gz::msgs::LaserScan scan;
    *scan.mutable_header()->mutable_stamp() = stamp;
    scan.set_frame(frame_id);
    scan.set_range_min(model.range.min);
    scan.set_range_max(model.range.max);
    scan.set_angle_min(model.theta.min);
    scan.set_angle_step(model.theta.inc);
    if(model.theta.size > 0)
    {
      scan.set_angle_max(model.theta.min
        + model.theta.inc * static_cast<float>(model.theta.size - 1));
    } else {
      scan.set_angle_max(model.theta.min);
    }
    scan.set_count(model.theta.size);
    scan.set_vertical_angle_min(0.0);
    scan.set_vertical_angle_max(0.0);
    scan.set_vertical_angle_step(0.0);
    scan.set_vertical_count(1);
    scan.mutable_ranges()->Reserve(static_cast<int>(ranges.size()));
    for(size_t i = 0; i < ranges.size(); ++i)
    {
      scan.add_ranges(static_cast<double>(ranges[i]));
    }
    // Publish() isn't const on gz::transport::Node::Publisher -- scan_pubs
    // must be a non-const reference (see this function's signature).
    for(auto &scan_pub : scan_pubs)
    {
      scan_pub.Publish(scan);
    }
  }
}

// Optional extra per-point data a simulate() call can produce alongside
// Ranges (see rmagine/simulation/SimulationResults.hpp) -- Classic's ROS 1
// PointCloud2 publisher could emit `ring`/normals/`obj_id`/`face_id` on
// top of x/y/z. nullptr means "not requested/available", and that field is
// omitted from the message entirely (not zero-filled) -- callers only pay
// for what they ask `RunAndPublish`'s ResT Bundle to compute.
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
  const gz::msgs::Time &stamp,
  const std::string &frame_id,
  std::vector<gz::transport::Node::Publisher> &points_pubs,
  const PointCloudExtras &extras = PointCloudExtras{})
{
  if(points_pubs.empty())
  {
    return;
  }

  using Field = gz::msgs::PointCloudPacked::Field;

  std::vector<std::pair<std::string, Field::DataType>> fields;
  fields.emplace_back("xyz", Field::FLOAT32);
  // ring: which row (vid) a point came from -- a rotating-scanner concept
  // (Spherical's phi rows), but well-defined identically for every model
  // type here (just the row index), so always published, not gated behind
  // model type the way LaserScan is.
  fields.emplace_back("ring", Field::UINT16);
  if(extras.normals)
  {
    fields.emplace_back("normal_x", Field::FLOAT32);
    fields.emplace_back("normal_y", Field::FLOAT32);
    fields.emplace_back("normal_z", Field::FLOAT32);
  }
  if(extras.object_ids)
  {
    fields.emplace_back("obj_id", Field::UINT32);
  }
  if(extras.face_ids)
  {
    fields.emplace_back("face_id", Field::UINT32);
  }

  gz::msgs::PointCloudPacked msg;
  gz::msgs::InitPointCloudPacked(msg, frame_id, false, fields);
  *msg.mutable_header()->mutable_stamp() = stamp;

  // InitPointCloudPacked() already set frame_id via a header "frame_id"
  // data entry (see its own implementation) -- nothing further needed here
  // for frame_id specifically.

  const uint32_t point_step = msg.point_step();
  uint32_t offset = 0;
  const uint32_t off_x = offset; offset += 3 * sizeof(float);
  const uint32_t off_ring = offset; offset += sizeof(uint16_t);
  uint32_t off_nx = 0, off_obj_id = 0, off_face_id = 0;
  if(extras.normals)
  {
    off_nx = offset; offset += 3 * sizeof(float);
  }
  if(extras.object_ids)
  {
    off_obj_id = offset; offset += sizeof(uint32_t);
  }
  if(extras.face_ids)
  {
    off_face_id = offset; offset += sizeof(uint32_t);
  }

  msg.set_height(static_cast<uint32_t>(model.getHeight()));
  msg.set_width(static_cast<uint32_t>(model.getWidth()));
  msg.set_is_bigendian(false);
  msg.set_is_dense(false);
  msg.set_row_step(msg.width() * point_step);
  msg.mutable_data()->resize(msg.width() * msg.height() * point_step);

  for(size_t vid = 0; vid < model.getHeight(); vid++)
  {
    for(size_t hid = 0; hid < model.getWidth(); hid++)
    {
      const unsigned int pid = model.getBufferId(vid, hid);
      const float range = ranges[pid];
      char* buff = &(*msg.mutable_data())[pid * point_step];
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

  for(auto &points_pub : points_pubs)
  {
    points_pub.Publish(msg);
  }
}

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_SENSOR_MODEL_PUBLISH_HPP
