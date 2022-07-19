#include <rmagine_gazebo_plugins/rmagine_embree_ros_gzplugin.h>

#include <iostream>


namespace gazebo
{

RmagineEmbreeROS::RmagineEmbreeROS()
:Base()
{
    std::cout << "[RmagineEmbreeROS] Construct" << std::endl;
}

RmagineEmbreeROS::~RmagineEmbreeROS()
{
    std::cout << "[RmagineEmbreeROS] Destroy" << std::endl;
}

void RmagineEmbreeROS::Load(
    sensors::SensorPtr _sensor, 
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    std::cout << "[RmagineEmbreeROS] Load!" << std::endl;
    m_spherical_sensor =
        std::dynamic_pointer_cast<sensors::RmagineEmbreeSpherical>(_sensor);

    if (!m_spherical_sensor)
    {
        gzerr << "RmagineEmbreeROS requires a RmagineEmbreeSpherical, RmagineEmbreePinhole, RmagineEmbreeO1Dn or RmagineEmbreeOnDn.\n";
        return;
    }

    // Connect to the sensor update event.
    m_update_conn = m_spherical_sensor->ConnectUpdated(
        std::bind(&RmagineEmbreeROS::OnUpdate, this));

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    m_nh.reset(new ros::NodeHandle(m_robot_namespace));

    m_topic_name = m_sdf->Get<std::string>("topic");
    m_frame_id = m_sdf->Get<std::string>("frame");

    // m_spherical_sensor->SetActive(true);
}

void RmagineEmbreeROS::OnUpdate()
{
    // std::cout << "[RmagineEmbreeROS] Update!" << std::endl;

    common::Time stamp_gz = m_spherical_sensor->stamp();
    rm::SphericalModel sensor_model = m_spherical_sensor->sensorModel();
    rm::Memory<float, rm::RAM> ranges = m_spherical_sensor->ranges();

    ros::Time stamp(stamp_gz.sec, stamp_gz.nsec);

    if(sensor_model.phi.size == 1)
    {
        if(!m_pub_laser)
        {
            if(m_nh)
            {
                m_pub_laser = std::make_shared<ros::Publisher>(
                        m_nh->advertise<sensor_msgs::LaserScan>(
                            m_topic_name, 1
                        ) 
                    );
            } else {
                // TODO error message
            }
        }

        // ready to build sensor_msgs::LaserScan msg
        sensor_msgs::LaserScan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = m_frame_id;

        msg.ranges.resize(ranges.size());

        msg.range_min = sensor_model.range.min;
        msg.range_max = sensor_model.range.max;
        msg.angle_min = sensor_model.theta.min;
        msg.angle_increment = sensor_model.theta.inc;

        for(size_t i=0; i<ranges.size(); i++)
        {
            msg.ranges[i] = ranges[i];
        }

        m_pub_laser->publish(msg);
    } else {
        std::cout << "[RmagineEmbreeROS] Could not make Laserscan. phi size 1 != " << sensor_model.phi.size << std::endl;
    }
}

GZ_REGISTER_SENSOR_PLUGIN(RmagineEmbreeROS)

} // namespace gazebo