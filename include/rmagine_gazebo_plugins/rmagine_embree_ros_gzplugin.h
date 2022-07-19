#ifndef GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <rmagine/noise/noise.h>
#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

#include <sensor_msgs/LaserScan.h>

#include <memory>

namespace rm = rmagine;

namespace gazebo
{

class RmagineEmbreeROS : public SensorPlugin
{
public:
    using Base = SensorPlugin;

    RmagineEmbreeROS();

    virtual ~RmagineEmbreeROS();

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:

    virtual void OnUpdate();

    sensors::RmagineEmbreeSphericalPtr m_spherical_sensor;

    event::ConnectionPtr m_update_conn;

    std::shared_ptr<ros::NodeHandle> m_nh;
    std::shared_ptr<ros::Publisher> m_pub_laser;

    std::string m_robot_namespace;

    std::string m_topic_name;
    std::string m_frame_id;

    sdf::ElementPtr m_sdf;
};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_ROS_PLUGIN_H