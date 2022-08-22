#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>
#include <boost/algorithm/string/replace.hpp>

#include <rmagine/noise/GaussianNoiseCuda.hpp>
#include <rmagine/noise/UniformDustNoiseCuda.hpp>
#include <rmagine/noise/RelGaussianNoiseCuda.hpp>

#include <gazebo/common/Console.hh>


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
    gzdbg << "[RmagineOptixSpherical] fetching parameters from sdf" << std::endl;

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

RmagineOptixSpherical::RmagineOptixSpherical()
:Base(sensors::RAY) // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RmagineOptixSpherical] Construct" << std::endl;
}

RmagineOptixSpherical::~RmagineOptixSpherical()
{
    gzdbg << "[RmagineOptixSpherical] Destroy" << std::endl;
}

void RmagineOptixSpherical::Load(const std::string& world_name)
{
    Base::Load(world_name);
    
    GZ_ASSERT(this->world != nullptr,
      "RaySensor did not get a valid World pointer");

    sdf::ElementPtr rayElem = this->sdf->GetElement("ray");

    // GET SENSOR MODEL
    if(rayElem->HasElement("scan"))
    {
        m_sensor_model = fetch_sensor_model(rayElem);
        sim_buffers.ranges.resize(m_sensor_model.size());
    }

    // GET NOISE MODEL
    if(rayElem->HasElement("noise"))
    {
        // has noise
        rm::NoiseCuda::Options opt = {};
        opt.estimated_memory_size = m_sensor_model.size();
        opt.max_range = m_sensor_model.range.max;

        sdf::ElementPtr noiseElem = rayElem->GetElement("noise");

        while(noiseElem)
        {
            std::string noise_type = noiseElem->Get<std::string>("type");
            if(noise_type == "gaussian")
            {
                gzdbg << "[RmagineOptixSpherical] init noise: 'gaussian'" << std::endl;
                
                float mean = 0.0;
                if(noiseElem->HasElement("mean"))
                {
                    mean = noiseElem->Get<float>("mean");
                }

                float stddev = noiseElem->Get<float>("stddev");

                rm::NoiseCudaPtr gaussian_noise = std::make_shared<rm::GaussianNoiseCuda>(
                    mean,
                    stddev,
                    opt
                );

                m_noise_models.push_back(gaussian_noise);

            } else if(noise_type == "uniform_dust") {
                gzdbg << "[RmagineOptixSpherical] init noise: 'uniform_dust'" << std::endl;
                
                float hit_prob = noiseElem->Get<float>("hit_prob");
                float return_prob = noiseElem->Get<float>("return_prob");

                rm::NoiseCudaPtr uniform_dust_noise = std::make_shared<rm::UniformDustNoiseCuda>(
                    hit_prob,
                    return_prob,
                    opt
                );

                m_noise_models.push_back(uniform_dust_noise);

            } else if(noise_type == "rel_gaussian") {
                gzdbg << "[RmagineOptixSpherical] init noise: 'rel_gaussian'" << std::endl;
                
                float mean = 0.0;
                float range_exp = 1.0;
                float stddev = noiseElem->Get<float>("stddev");

                if(noiseElem->HasElement("mean"))
                {
                    mean = noiseElem->Get<float>("mean");
                }

                if(noiseElem->HasElement("range_exp"))
                {
                    range_exp = noiseElem->Get<float>("range_exp");
                }

                rm::NoiseCudaPtr gaussian_noise = std::make_shared<rm::RelGaussianNoiseCuda>(
                    mean,
                    stddev,
                    range_exp,
                    opt
                );

                m_noise_models.push_back(gaussian_noise);
            } else {
                gzwarn << "[RmagineOptixSpherical] WARNING: SDF noise type '" << noise_type << "' unknown. skipping." << std::endl;
            }

            noiseElem = noiseElem->GetNextElement("noise");
        }
    }

    // COMPUTE
    if(rayElem->HasElement("compute"))
    {
        sdf::ElementPtr computeElem = rayElem->GetElement("compute");
        
        if(computeElem->HasElement("normals"))
        {
            if(computeElem->Get<bool>("normals"))
            {
                sim_buffers.normals.resize(m_sensor_model.size());
            }
        }

        if(computeElem->HasElement("object_ids"))
        {
            if(computeElem->Get<bool>("object_ids"))
            {
                sim_buffers.object_ids.resize(m_sensor_model.size());
            }
        }

        if(computeElem->HasElement("geom_ids"))
        {
            if(computeElem->Get<bool>("geom_ids"))
            {
                sim_buffers.geom_ids.resize(m_sensor_model.size());
            }
        }

        if(computeElem->HasElement("face_ids"))
        {
            if(computeElem->Get<bool>("face_ids"))
            {
                sim_buffers.face_ids.resize(m_sensor_model.size());
            }
        }
    }

    this->parentEntity = this->world->EntityByName(this->ParentName());

    GZ_ASSERT(this->parentEntity != nullptr,
      "Unable to get the parent entity.");

    auto pose = Pose();
    m_Tsb = to_rm(pose);

    // std::cout << "[RmagineOptixSpherical] advertising topic " << this->Topic() << std::endl;
    
    if(m_gz_publish)
    {
        this->scanPub =
            this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

        if(!this->scanPub || !this->scanPub->HasConnections())
        {
            gzwarn << "[RmagineOptixSpherical] Gazebo internal publishing failed. Reason: ";

            if(!this->scanPub)
            {
                gzwarn << "- Reason: No scanPub" << std::endl;
            } else {
                gzwarn << "- Reason: No connections" << std::endl;
            }
        }
    }
}

void RmagineOptixSpherical::Init()
{
    Base::Init();
    this->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

std::string RmagineOptixSpherical::Topic() const
{
    std::string topicName = "~/";
    topicName += this->ParentName() + "/" + this->Name() + "/scan";
    boost::replace_all(topicName, "::", "/");

    return topicName;
}

bool RmagineOptixSpherical::IsActive() const
{
    return Sensor::IsActive() ||
        (this->scanPub && this->scanPub->HasConnections());
}

void RmagineOptixSpherical::setMap(rm::OptixMapPtr map)
{
    m_map = map;
    

    if(m_sphere_sim)
    {
        m_sphere_sim->setMap(map);
    } else {
        m_sphere_sim = std::make_shared<rm::SphereSimulatorOptix>(map);
        m_sphere_sim->setTsb(m_Tsb);
        m_sphere_sim->setModel(m_sensor_model);
    }

    if(m_waiting_for_map)
    {
        std::cout << "[RmagineOptixSpherical] RmagineOptixMap found." << std::endl;
    }

    m_waiting_for_map = false;
}

void RmagineOptixSpherical::setLock(std::shared_ptr<std::shared_mutex> mutex)
{
    m_map_mutex = mutex;
}

bool RmagineOptixSpherical::UpdateImpl(const bool _force)
{   
    if(m_sphere_sim)
    {
        IGN_PROFILE("RmagineOptixSpherical::UpdateImpl");
        IGN_PROFILE_BEGIN("Update");

        auto pose = parentEntity->WorldPose();
        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = to_rm(pose);
        rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_ = Tbms;

        if(m_map_mutex)
        {
            m_map_mutex->lock_shared();
        }

        m_sphere_sim->simulate(Tbms_, sim_buffers);
        
        if(m_map_mutex)
        {
            m_map_mutex->unlock_shared();
        }

        // apply noise
        for(auto noise_model : m_noise_models)
        {
            noise_model->apply(sim_buffers.ranges);
        }

        this->lastMeasurementTime = this->world->SimTime();
        updateScanMsg(sim_buffers.ranges);
        IGN_PROFILE_END();

        // std::cout << "[RmagineOptixSpherical] Simulated " << m_ranges.size() << " ranges" << std::endl;
        
        if(m_gz_publish)
        {
            IGN_PROFILE_BEGIN("Publish");
            if (this->scanPub && this->scanPub->HasConnections())
            {
                this->scanPub->Publish(this->laserMsg);
            } else {
                gzwarn << "[RmagineOptixSpherical] Publishing failed. " << std::endl;
            }
            IGN_PROFILE_END();
        }

        return true;
    } else {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RmagineOptixSpherical] waiting for RmagineOptixMap..." << std::endl;
            m_waiting_for_map = true;
        }
    }
    
    return false;
}

void RmagineOptixSpherical::updateScanMsg(
    const rm::MemoryView<float, rm::VRAM_CUDA>& ranges_)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    
    // download
    rm::Memory<float, rm::RAM> ranges = ranges_;

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

void RmagineOptixSpherical::Fini()
{
    Base::Fini();
    this->scanPub.reset();
}

GZ_REGISTER_STATIC_SENSOR("rmagine_optix_spherical", RmagineOptixSpherical)

} // namespace sensors

} // namespace gazebo