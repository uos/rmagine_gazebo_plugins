#ifndef GAZEBO_RMAGINE_OPTIX_ROS_PLUGIN_H
#define GAZEBO_RMAGINE_OPTIX_ROS_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <unordered_map>

namespace rm = rmagine;

namespace gazebo
{

struct Publisher {
    std::string msg_type;
    std::string topic;
    std::shared_ptr<ros::Publisher> pub;
};

class RmagineOptixROS : public SensorPlugin
{
public:
    using Base = SensorPlugin;

    RmagineOptixROS();

    virtual ~RmagineOptixROS();

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:

    void parseOutputs(sdf::ElementPtr outputs);

    virtual void OnUpdate();

    sensors::RmagineOptixSphericalPtr m_spherical_sensor;

    event::ConnectionPtr m_update_conn;

    std::shared_ptr<ros::NodeHandle> m_nh;

    std::unordered_map<std::string, Publisher> m_pubs;

    std::string m_robot_namespace;

    // std::string m_laser_topic_name;
    std::string m_frame_id;

    // unordered pointclouds unique by topic name
    std::unordered_set<std::string> m_pcl2_unordered;

    sdf::ElementPtr m_sdf;
};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_OPTIX_ROS_PLUGIN_H