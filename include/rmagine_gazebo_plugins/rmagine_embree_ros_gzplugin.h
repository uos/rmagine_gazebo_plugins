#ifndef GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

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

class RmagineEmbreeROS : public SensorPlugin
{
public:
    using Base = SensorPlugin;

    RmagineEmbreeROS();

    virtual ~RmagineEmbreeROS();

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:

    void parseOutputs(sdf::ElementPtr outputs);

    virtual void OnUpdate();

    sensors::RmagineEmbreeSphericalPtr m_spherical_sensor;

    event::ConnectionPtr m_update_conn;

    std::shared_ptr<ros::NodeHandle> m_nh;

    std::unordered_map<std::string, Publisher> m_pubs;

    std::string m_robot_namespace;

    // std::string m_laser_topic_name;
    std::string m_frame_id;

    sdf::ElementPtr m_sdf;
};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H