#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/noise.h>

using namespace std::placeholders;

namespace gazebo
{

namespace sensors
{


static rm::Transform to_rm(const ignition::math::Pose3d& pose)
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

static rm::SphericalModel fetch_sensor_model(sdf::ElementPtr rayElem)
{
    std::cout << "[RmagineEmbreeSpherical] fetching parameters from sdf" << std::endl;

    sdf::ElementPtr scanElem = rayElem->GetElement("scan");

    rm::SphericalModel sensor_model;

    if(scanElem->HasElement("horizontal"))
    {
        sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
        sensor_model.theta.min = horElem->Get<float>("min_angle");
        sensor_model.theta.inc = horElem->Get<float>("increment");
        sensor_model.theta.size = horElem->Get<int>("samples");
    } else {
        // scanner scans only one column
        sensor_model.theta.min = 0.0;
        sensor_model.theta.inc = 1.0;
        sensor_model.theta.size = 1;
    }

    if(scanElem->HasElement("vertical"))
    {
        sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
        sensor_model.phi.min = vertElem->Get<float>("min_angle");
        sensor_model.phi.inc = vertElem->Get<float>("increment");
        sensor_model.phi.size = vertElem->Get<int>("samples");
    } else {
        // scanner scan only one row
        sensor_model.phi.min = 0.0;
        sensor_model.phi.inc = 1.0;
        sensor_model.phi.size = 1;
    }

    sdf::ElementPtr rangeElem = rayElem->GetElement("range");
    sensor_model.range.min = rangeElem->Get<float>("min");
    sensor_model.range.max = rangeElem->Get<float>("max");

    return sensor_model;
}



RmagineEmbreeSpherical::RmagineEmbreeSpherical()
:Base(sensors::RAY) // if sensor is base class: Base(sensors::RAY)
{
    std::cout << "[RmagineEmbreeSpherical] Construct" << std::endl;
}

RmagineEmbreeSpherical::~RmagineEmbreeSpherical()
{
    std::cout << "[RmagineEmbreeSpherical] Destroy" << std::endl;
}

void RmagineEmbreeSpherical::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "[RmagineEmbreeSpherical] Load " << std::endl;

    

    GZ_ASSERT(this->world != nullptr,
      "RaySensor did not get a valid World pointer");

    sdf::ElementPtr rayElem = this->sdf->GetElement("ray");

    // GET SENSOR MODEL
    if(rayElem->HasElement("scan"))
    {
        m_sensor_model = fetch_sensor_model(rayElem);
        m_ranges.resize(m_sensor_model.size());
    }

    // GET NOISE MODEL
    if(rayElem->HasElement("noise"))
    {
        // has noise
        sdf::ElementPtr noiseElem = rayElem->GetElement("noise");

        std::string noise_type = noiseElem->Get<std::string>("type");
        if(noise_type == "gaussian")
        {
            m_noise_type = NoiseType::GAUSSIAN;
            
            if(noiseElem->HasElement("mean"))
            {
                m_noise_mean = noiseElem->Get<float>("mean");
            }

            m_noise_stddev = noiseElem->Get<float>("stddev");
        } else {
            std::cout << "[RmagineEmbreeSpherical] WARNING: SDF noise type '" << noise_type << "' unknown. skipping." << std::endl;
        }
        
        
    }

    this->parentEntity = this->world->EntityByName(this->ParentName());

    GZ_ASSERT(this->parentEntity != nullptr,
      "Unable to get the parent entity.");

    auto pose = Pose();
    m_Tsb = to_rm(pose);

    std::cout << "[RmagineEmbreeSpherical] advertising topic " << this->Topic() << std::endl;
    
    if(m_gz_publish)
    {
        this->scanPub =
        this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

        if (!this->scanPub || !this->scanPub->HasConnections())
        {
            std::cout << "[RmagineEmbreeSpherical] Gazebo internal publishing failed. Reason: ";

            if(!this->scanPub)
            {
                std::cout << " No scanPub" << std::endl;
            } else {
                std::cout << " No connections" << std::endl;
            }
        }
    }
}

void RmagineEmbreeSpherical::Init()
{
    Base::Init();
    this->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

std::string RmagineEmbreeSpherical::Topic() const
{
    std::string topicName = "~/";
    topicName += this->ParentName() + "/" + this->Name() + "/scan";
    boost::replace_all(topicName, "::", "/");

    return topicName;
}

bool RmagineEmbreeSpherical::IsActive() const
{
    return Sensor::IsActive() ||
        (this->scanPub && this->scanPub->HasConnections());
}

void RmagineEmbreeSpherical::setMap(rm::EmbreeMapPtr map)
{
    m_map = map;
    
    if(m_sphere_sim)
    {
        m_sphere_sim->setMap(map);

    } else {
        m_sphere_sim = std::make_shared<rm::SphereSimulatorEmbree>(map);
        m_sphere_sim->setTsb(m_Tsb);
        m_sphere_sim->setModel(m_sensor_model);
    }

    m_waiting_for_map = false;
}

void RmagineEmbreeSpherical::setLock(std::shared_ptr<std::shared_mutex> mutex)
{
    m_map_mutex = mutex;
}

bool RmagineEmbreeSpherical::UpdateImpl(const bool _force)
{   
    if(m_sphere_sim)
    {
        IGN_PROFILE("RmagineEmbreeSpherical::UpdateImpl");
        IGN_PROFILE_BEGIN("Update");

        auto pose = parentEntity->WorldPose();
        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = to_rm(pose);
        
        if(m_pre_alloc_mem)
        {
            if(m_map_mutex)
            {
                m_map_mutex->lock_shared();
            }
            m_sphere_sim->simulateRanges(Tbms, m_ranges);
            if(m_map_mutex)
            {
                m_map_mutex->unlock_shared();
            }
        } else {
            if(m_map_mutex)
            {
                m_map_mutex->lock_shared();
            }
            m_ranges = m_sphere_sim->simulateRanges(Tbms);
            if(m_map_mutex)
            {
                m_map_mutex->unlock_shared();
            }
        }

        // apply noise
        if(m_noise_type == NoiseType::GAUSSIAN)
        {
            rm::GaussianNoise(m_noise_mean, m_noise_stddev).apply(m_ranges);
        }
        this->lastMeasurementTime = this->world->SimTime();
        updateScanMsg(m_ranges);

        IGN_PROFILE_END();

        // std::cout << "[RmagineEmbreeSpherical] Simulated " << m_ranges.size() << " ranges" << std::endl;
        
        if(m_gz_publish)
        {
            IGN_PROFILE_BEGIN("Publish");
            if (this->scanPub && this->scanPub->HasConnections())
            {
                this->scanPub->Publish(this->laserMsg);
            } else {
                 std::cout << "[RmagineEmbreeSpherical] Publishing failed. " << std::endl;
            }
            IGN_PROFILE_END();
        }

        return true;
    } else {
        if(!m_waiting_for_map)
        {
            std::cout << "[RmagineEmbreeSpherical] waiting for RmagineEmbreeMap..." << std::endl;
            m_waiting_for_map = true;
        }
    }
    
    return false;
}

void RmagineEmbreeSpherical::updateScanMsg(rm::MemoryView<float> ranges)
{
        std::lock_guard<std::mutex> lock(this->mutex);

        msgs::Set(this->laserMsg.mutable_time(),
            this->lastMeasurementTime);

        msgs::LaserScan *scan = this->laserMsg.mutable_scan();

        msgs::Set(scan->mutable_world_pose(),
            this->pose + this->parentEntity->WorldPose());
        
        scan->set_angle_min(m_sensor_model.theta.min);
        scan->set_angle_max(m_sensor_model.theta.max());
        scan->set_angle_step(m_sensor_model.theta.inc);
        scan->set_count(m_sensor_model.theta.size);

        scan->set_vertical_angle_min(m_sensor_model.phi.min);
        scan->set_vertical_angle_max(m_sensor_model.phi.max());
        scan->set_vertical_angle_step(m_sensor_model.phi.inc);
        scan->set_vertical_count(m_sensor_model.phi.size);

        scan->set_range_min(m_sensor_model.range.min);
        scan->set_range_max(m_sensor_model.range.max);

        scan->clear_ranges();
        scan->clear_intensities();

        // vertical direction
        for (unsigned int vid = 0; vid < m_sensor_model.phi.size; ++vid)
        {   
            // horizontal direction
            for (unsigned int hid = 0; hid < m_sensor_model.theta.size; ++hid)
            {
                unsigned int meas_id = m_sensor_model.getBufferId(vid, hid);
                double range;
                range = ranges[meas_id];
                if(range >= m_sensor_model.range.max)
                {
                    range = ignition::math::INF_D;
                } else if(range <= m_sensor_model.range.min) {
                    range = -ignition::math::INF_D;
                }

                scan->add_ranges(range);
            }
        }
}

void RmagineEmbreeSpherical::Fini()
{
    Base::Fini();
    this->scanPub.reset();
}

GZ_REGISTER_STATIC_SENSOR("rmagine_embree_spherical", RmagineEmbreeSpherical)

} // namespace sensors

} // namespace gazebo