#include <rmagine_gazebo_plugins/rmagine_embree_ros_gzplugin.h>

#include <iostream>


namespace gazebo
{

RmagineEmbreeROS::RmagineEmbreeROS()
:Base()
{
    // ROS_INFO_STREAM("[RmagineEmbreeROS] Construct");
}

RmagineEmbreeROS::~RmagineEmbreeROS()
{
    // std::cout << "[RmagineEmbreeROS] Destroy" << std::endl;
}

void RmagineEmbreeROS::parseOutputs(sdf::ElementPtr outputs)
{
    // std::cout << "[RmagineEmbreeROS::parseOutputs]" << std::endl;

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

            if(pub.msg_type == "sensor_msgs/PointCloud2")
            {
                pub.pub = std::make_shared<ros::Publisher>(
                        m_nh->advertise<sensor_msgs::PointCloud2>(
                            pub.topic, 1
                        ) 
                    );

                if(it->HasElement("ordered"))
                {
                    bool ordered = it->Get<bool>("ordered");
                    
                    if(!ordered)
                    {
                        m_pcl2_unordered.insert(pub.topic);
                    }
                }
            }

            if(pub.pub)
            {
                m_pubs[name] = pub;
            } 
        }

        it = it->GetNextElement();
    }
}

void RmagineEmbreeROS::Load(
    sensors::SensorPtr _sensor, 
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

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

    ROS_INFO_STREAM("[RmagineEmbreeROS] Loaded.");
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
                ROS_WARN_STREAM("[RmagineEmbreeROS] Could not make Laserscan. phi size 1 != " << sensor_model.phi.size);
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

        if(pub.msg_type == "sensor_msgs/PointCloud2")
        {
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = m_frame_id;
            
            msg.fields.resize(4);

            uint32_t total_bytes = 0;

            sensor_msgs::PointField field_x;
            field_x.name = "x";
            field_x.offset = 0;
            field_x.datatype = sensor_msgs::PointField::FLOAT32;
            field_x.count = 1;

            sensor_msgs::PointField field_y;
            field_y.name = "y";
            field_y.offset = 4;
            field_y.datatype = sensor_msgs::PointField::FLOAT32;
            field_y.count = 1;

            sensor_msgs::PointField field_z;
            field_z.name = "z";
            field_z.offset = 8;
            field_z.datatype = sensor_msgs::PointField::FLOAT32;
            field_z.count = 1;

            sensor_msgs::PointField field_ring;
            field_ring.name = "ring";
            field_ring.offset = 12;
            field_ring.datatype = sensor_msgs::PointField::UINT16;
            field_ring.count = 1;

            msg.fields[0] = field_x;
            msg.fields[1] = field_y;
            msg.fields[2] = field_z;
            msg.fields[3] = field_ring;

            struct MyPoint
            {
                rm::Vector3 p;
                uint16_t ring;
            };

            msg.point_step = sizeof(MyPoint);
            msg.is_dense = false;

            // ordered
            if(m_pcl2_unordered.find(pub.topic) == m_pcl2_unordered.end())
            {
                msg.height = sensor_model.phi.size;
                msg.width = sensor_model.theta.size;

                msg.data.resize(msg.width * msg.height * msg.point_step);

                MyPoint* data = reinterpret_cast<MyPoint*>(&msg.data[0]);

                for(size_t vid = 0; vid < sensor_model.getHeight(); vid++)
                {
                    for(size_t hid = 0; hid < sensor_model.getWidth(); hid++)
                    {
                        const unsigned int pid = sensor_model.getBufferId(vid, hid);
                        const float range = ranges[pid];
                        
                        if(sensor_model.range.inside(range))
                        {
                            data[pid].p = sensor_model.getDirection(vid, hid) * range;
                        } else {
                            // ordered pcl: fill with nans
                            data[pid].p = rm::Vector3::NaN();
                        }

                        data[pid].ring = vid;
                    }
                }
            } else {
                // unordered
                msg.height = 1;
                // unclear
                msg.is_dense = true;

                std::vector<MyPoint> data;
                data.reserve(sensor_model.theta.size * sensor_model.phi.size);

                for(size_t vid = 0; vid < sensor_model.getHeight(); vid++)
                {
                    for(size_t hid = 0; hid < sensor_model.getWidth(); hid++)
                    {
                        const unsigned int pid = sensor_model.getBufferId(vid, hid);
                        const float range = ranges[pid];
                        
                        if(sensor_model.range.inside(range))
                        {
                            MyPoint entry;
                            entry.p = sensor_model.getDirection(vid, hid) * range;
                            entry.ring = vid;
                            data.push_back(entry);
                        }
                    }
                }
                
                msg.width = data.size();
                // copy bytewise
                msg.data.resize(msg.width * msg.point_step);
                MyPoint* data_raw = reinterpret_cast<MyPoint*>(&msg.data[0]);
                std::copy(data.begin(), data.end(), data_raw);
            }

            msg.row_step = msg.width * msg.point_step;

            pub.pub->publish(msg);
        }
    }

}

GZ_REGISTER_SENSOR_PLUGIN(RmagineEmbreeROS)

} // namespace gazebo