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

void RmagineEmbreeROS::parseOutputs(sdf::ElementPtr outputs)
{

    std::cout << "[RmagineEmbreeROS::parseOutputs]" << std::endl;

    auto it = outputs->GetFirstElement();

    while(it)
    {
        if(it->GetName() == "output")
        {
            sdf::ParamPtr nameParam = it->GetAttribute("name");
            std::string name = nameParam->GetAsString();

            Publisher pub;
            pub.msg_type = it->Get<std::string>("msg");
            pub.topic = it->Get<std::string>("topic");

            if(pub.msg_type == "sensor_msgs/LaserScan")
            {
                pub.pub = std::make_shared<ros::Publisher>(
                        m_nh->advertise<sensor_msgs::LaserScan>(
                            pub.topic, 1
                        )
                    );
            }

            if(pub.msg_type == "sensor_msgs/PointCloud")
            {
                pub.pub = std::make_shared<ros::Publisher>(
                        m_nh->advertise<sensor_msgs::PointCloud>(
                            pub.topic, 1
                        ) 
                    );
            }

            if(pub.pub)
            {
                m_pubs[name] = pub;
            } 
        }

        

        it = it->GetNextElement();
    }

    // if(outputs->HasElement("laser"))
    // {
    //     sdf::ElementPtr laserElem = outputs->GetElement("laser");
    //     std::string topic = laserElem->Get<std::string>("topic");
    //     m_pub_laser = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::LaserScan>(
    //                         topic, 1
    //                     ) 
    //                 );
    // }

    // if(outputs->HasElement("pcl"))
    // {
    //     sdf::ElementPtr pclElem = outputs->GetElement("pcl");
    //     std::string topic = pclElem->Get<std::string>("topic");
    //     m_pub_pcl = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::PointCloud>(
    //                         topic, 1
    //                     ) 
    //                 );
    // }

    // if(outputs->HasElement("pcl2"))
    // {
    //     sdf::ElementPtr pcl2Elem = outputs->GetElement("pcl2");
    //     std::string topic = pcl2Elem->Get<std::string>("topic");
    //     m_pub_pcl2 = std::make_shared<ros::Publisher>(
    //                     m_nh->advertise<sensor_msgs::PointCloud2>(
    //                         topic, 1
    //                     ) 
    //                 );
    // }
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

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    m_nh.reset(new ros::NodeHandle(m_robot_namespace));

    m_frame_id = m_sdf->Get<std::string>("frame");

    sdf::ElementPtr outputsElem = m_sdf->GetElement("outputs");
    parseOutputs(outputsElem);

    // Connect to the sensor update event.
    m_update_conn = m_spherical_sensor->ConnectUpdated(
        std::bind(&RmagineEmbreeROS::OnUpdate, this));
}

void RmagineEmbreeROS::OnUpdate()
{
    // std::cout << "[RmagineEmbreeROS] Update!" << std::endl;

    common::Time stamp_gz = m_spherical_sensor->stamp();
    rm::SphericalModel sensor_model = m_spherical_sensor->sensorModel();
    rm::Memory<float, rm::RAM> ranges = m_spherical_sensor->ranges();

    ros::Time stamp(stamp_gz.sec, stamp_gz.nsec);

    for(auto elem : m_pubs)
    {
        auto pub = elem.second;
        if(pub.msg_type == "sensor_msgs/LaserScan")
        {
            if(sensor_model.phi.size == 1)
            {
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

                pub.pub->publish(msg);
            } else {
                std::cout << "[RmagineEmbreeROS] Could not make Laserscan. phi size 1 != " << sensor_model.phi.size << std::endl;
            }
        }

        if(pub.msg_type == "sensor_msgs/PointCloud")
        {
            sensor_msgs::PointCloud msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = m_frame_id;
            
            msg.points.reserve(ranges.size());
            for(size_t vid = 0; vid < sensor_model.getHeight(); vid++)
            {
                for(size_t hid = 0; hid < sensor_model.getWidth(); hid++)
                {
                    const unsigned int pid = sensor_model.getBufferId(vid, hid);
                    const float range = ranges[pid];
                    if(sensor_model.range.inside(range))
                    {
                        rm::Vector p = sensor_model.getDirection(vid, hid) * range;
                        geometry_msgs::Point32 p_ros;
                        p_ros.x = p.x;
                        p_ros.y = p.y;
                        p_ros.z = p.z;
                        msg.points.push_back(p_ros);
                    }
                }
            }

            pub.pub->publish(msg);
        }
    }

}

GZ_REGISTER_SENSOR_PLUGIN(RmagineEmbreeROS)

} // namespace gazebo