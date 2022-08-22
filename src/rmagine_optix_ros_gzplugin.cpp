#include <rmagine_gazebo_plugins/rmagine_optix_ros_gzplugin.h>

#include <iostream>


namespace gazebo
{

RmagineOptixROS::RmagineOptixROS()
:Base()
{
    std::cout << "[RmagineOptixROS] Construct" << std::endl;
}

RmagineOptixROS::~RmagineOptixROS()
{
    std::cout << "[RmagineOptixROS] Destroy" << std::endl;
}

void RmagineOptixROS::parseOutputs(sdf::ElementPtr outputs)
{
    std::cout << "[RmagineOptixROS::parseOutputs]" << std::endl;

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

void RmagineOptixROS::Load(
    sensors::SensorPtr _sensor, 
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    std::cout << "[RmagineOptixROS] Load!" << std::endl;
    m_spherical_sensor =
        std::dynamic_pointer_cast<sensors::RmagineOptixSpherical>(_sensor);

    if (!m_spherical_sensor)
    {
        gzerr << "RmagineOptixROS requires a RmagineOptixSpherical, RmagineOptixPinhole, RmagineOptixO1Dn or RmagineOptixOnDn.\n";
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
        std::bind(&RmagineOptixROS::OnUpdate, this));
}

void RmagineOptixROS::OnUpdate()
{
    // std::cout << "[RmagineOptixROS] Update!" << std::endl;
    common::Time stamp_gz = m_spherical_sensor->stamp();
    rm::SphericalModel sensor_model = m_spherical_sensor->sensorModel();
    
    // download
    rm::Memory<float, rm::RAM> ranges = m_spherical_sensor->sim_buffers.ranges;
    rm::Memory<rm::Vector3, rm::RAM> normals = m_spherical_sensor->sim_buffers.normals;
    const bool has_normals = normals.size() > 0;
    rm::Memory<unsigned int, rm::RAM> object_ids = m_spherical_sensor->sim_buffers.object_ids;
    const bool has_object_ids = object_ids.size() > 0;
    rm::Memory<unsigned int, rm::RAM> geom_ids = m_spherical_sensor->sim_buffers.geom_ids;
    const bool has_geom_ids = geom_ids.size() > 0;
    rm::Memory<unsigned int, rm::RAM> face_ids = m_spherical_sensor->sim_buffers.face_ids;
    const bool has_face_ids = face_ids.size() > 0;

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
                std::cout << "[RmagineOptixROS] Could not make Laserscan. phi size 1 != " << sensor_model.phi.size << std::endl;
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
            
            msg.fields.resize(0);

            uint32_t total_bytes = 0;

            sensor_msgs::PointField field_x;
            field_x.name = "x";
            field_x.offset = 0;
            field_x.datatype = sensor_msgs::PointField::FLOAT32;
            field_x.count = 1;
            msg.fields.push_back(field_x);
            total_bytes += sizeof(float);

            sensor_msgs::PointField field_y;
            field_y.name = "y";
            field_y.offset = total_bytes;
            field_y.datatype = sensor_msgs::PointField::FLOAT32;
            field_y.count = 1;
            msg.fields.push_back(field_y);
            total_bytes += sizeof(float);

            sensor_msgs::PointField field_z;
            field_z.name = "z";
            field_z.offset = total_bytes;
            field_z.datatype = sensor_msgs::PointField::FLOAT32;
            field_z.count = 1;
            msg.fields.push_back(field_z);
            total_bytes += sizeof(float);

            sensor_msgs::PointField field_ring;
            field_ring.name = "ring";
            field_ring.offset = total_bytes;
            field_ring.datatype = sensor_msgs::PointField::UINT32;
            field_ring.count = 1;
            msg.fields.push_back(field_ring);
            total_bytes += sizeof(uint32_t);

            if(has_normals)
            {
                sensor_msgs::PointField field_nx;
                field_nx.name = "nx";
                field_nx.offset = total_bytes;
                field_nx.datatype = sensor_msgs::PointField::FLOAT32;
                field_nx.count = 1;
                msg.fields.push_back(field_nx);
                total_bytes += sizeof(float);

                sensor_msgs::PointField field_ny;
                field_ny.name = "ny";
                field_ny.offset = total_bytes;
                field_ny.datatype = sensor_msgs::PointField::FLOAT32;
                field_ny.count = 1;
                msg.fields.push_back(field_ny);
                total_bytes += sizeof(float);

                sensor_msgs::PointField field_nz;
                field_nz.name = "nz";
                field_nz.offset = total_bytes;
                field_nz.datatype = sensor_msgs::PointField::FLOAT32;
                field_nz.count = 1;
                msg.fields.push_back(field_nz);
                total_bytes += sizeof(float);
            }

            if(has_object_ids)
            {
                sensor_msgs::PointField field_obj_id;
                field_obj_id.name = "obj_id";
                field_obj_id.offset = total_bytes;
                field_obj_id.datatype = sensor_msgs::PointField::UINT32;
                field_obj_id.count = 1;
                msg.fields.push_back(field_obj_id);
                total_bytes += sizeof(uint32_t);
            }

            if(has_geom_ids)
            {
                sensor_msgs::PointField field_geom_id;
                field_geom_id.name = "geom_id";
                field_geom_id.offset = total_bytes;
                field_geom_id.datatype = sensor_msgs::PointField::UINT32;
                field_geom_id.count = 1;
                msg.fields.push_back(field_geom_id);
                total_bytes += sizeof(uint32_t);
            }

            if(has_face_ids)
            {
                sensor_msgs::PointField field_face_id;
                field_face_id.name = "face_id";
                field_face_id.offset = total_bytes;
                field_face_id.datatype = sensor_msgs::PointField::UINT32;
                field_face_id.count = 1;
                msg.fields.push_back(field_face_id);
                total_bytes += sizeof(uint32_t);
            }

            msg.point_step = total_bytes;
            

            // ordered
            if(m_pcl2_unordered.find(pub.topic) == m_pcl2_unordered.end())
            {
                msg.is_dense = false;
                
                msg.height = sensor_model.phi.size;
                msg.width = sensor_model.theta.size;

                msg.data.resize(msg.width * msg.height * msg.point_step);

                for(size_t vid = 0; vid < sensor_model.getHeight(); vid++)
                {
                    for(size_t hid = 0; hid < sensor_model.getWidth(); hid++)
                    {
                        const unsigned int pid = sensor_model.getBufferId(vid, hid);
                        const float range = ranges[pid];

                        uint8_t* buff = &msg.data[pid * msg.point_step];
                        
                        rm::Vector3* p = reinterpret_cast<rm::Vector3*>(buff);
                        if(sensor_model.range.inside(range))
                        {
                            *p = sensor_model.getDirection(vid, hid) * range;
                            buff += sizeof(rm::Vector3);

                            uint32_t* ring = reinterpret_cast<uint32_t*>(buff);
                            *ring = vid;
                            buff += sizeof(uint32_t);

                            if(has_normals)
                            {
                                rm::Vector3* n = reinterpret_cast<rm::Vector3*>(buff);
                                *n = normals[pid];
                                buff += sizeof(rm::Vector3);
                            }

                            if(has_object_ids)
                            {
                                uint32_t* obj_id = reinterpret_cast<uint32_t*>(buff);
                                *obj_id = object_ids[pid];
                                buff += sizeof(uint32_t);
                            }

                            if(has_geom_ids)
                            {
                                uint32_t* geom_id = reinterpret_cast<uint32_t*>(buff);
                                *geom_id = geom_ids[pid];
                                buff += sizeof(uint32_t);
                            }

                            if(has_face_ids)
                            {
                                uint32_t* face_id = reinterpret_cast<uint32_t*>(buff);
                                *face_id = face_ids[pid];
                                buff += sizeof(uint32_t);
                            }
                            
                        } else {
                            // ordered pcl: fill with nans
                            *p = rm::Vector3::NaN();
                        }

                        // data[pid].ring = vid;
                    }
                }
            } else {
                // unordered
                // msg.height = 1;
                // // unclear
                // msg.is_dense = true;

                // std::vector<MyPoint> data;
                // data.reserve(sensor_model.theta.size * sensor_model.phi.size);

                // for(size_t vid = 0; vid < sensor_model.getHeight(); vid++)
                // {
                //     for(size_t hid = 0; hid < sensor_model.getWidth(); hid++)
                //     {
                //         const unsigned int pid = sensor_model.getBufferId(vid, hid);
                //         const float range = ranges[pid];
                        
                //         if(sensor_model.range.inside(range))
                //         {
                //             MyPoint entry;
                //             entry.p = sensor_model.getDirection(vid, hid) * range;
                //             entry.ring = vid;
                //             data.push_back(entry);
                //         }
                //     }
                // }
                
                // msg.width = data.size();
                // // copy bytewise
                // msg.data.resize(msg.width * msg.point_step);
                // MyPoint* data_raw = reinterpret_cast<MyPoint*>(&msg.data[0]);
                // std::copy(data.begin(), data.end(), data_raw);
            }

            msg.row_step = msg.width * msg.point_step;

            pub.pub->publish(msg);
        }
    }

}

GZ_REGISTER_SENSOR_PLUGIN(RmagineOptixROS)

} // namespace gazebo