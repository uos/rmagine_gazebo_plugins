#ifndef GAZEBO_RMAGINE_OPTIX_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_OPTIX_SPHERICAL_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorOptix.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>

#include <rmagine/noise/NoiseCuda.hpp>


namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RmagineOptixSpherical : public Sensor
{
public:
    using Base = Sensor;

    RmagineOptixSpherical();

    virtual ~RmagineOptixSpherical();

    virtual void Load(const std::string& world_name) override;

    virtual void Init() override;
    
    virtual std::string Topic() const override;

    virtual bool IsActive() const override;

    void setMap(rm::OptixMapPtr map);

    void setLock(std::shared_ptr<std::shared_mutex> mutex);

    void updateScanMsg(const rm::MemoryView<float, rm::VRAM_CUDA>& ranges);


    inline common::Time stamp() const 
    {
        return lastMeasurementTime;
    }

    inline rm::SphericalModel sensorModel() const
    {
        return m_sensor_model;
    }

    rm::IntAttrAny<rm::VRAM_CUDA> sim_buffers;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    virtual void Fini() override;

    bool m_needs_update = false;

    rm::SphericalModel m_sensor_model;
    rm::Transform m_Tsb;


    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rm::OptixMapPtr m_map;
    rm::SphereSimulatorOptixPtr m_sphere_sim;

    

    // rm::Memory<float, rm::VRAM_CUDA> m_ranges;

    bool m_gz_publish = false;

    std::vector<rm::NoiseCudaPtr> m_noise_models;

// private:

    /// \brief Parent entity pointer
    physics::EntityPtr parentEntity;

    /// \brief Publisher for the scans
    transport::PublisherPtr scanPub;

    /// \brief Laser message.
    msgs::LaserScanStamped laserMsg;

    /// \brief Mutex to protect laserMsg
    std::mutex mutex;

    bool m_waiting_for_map = false;
};

using RmagineOptixSphericalPtr = std::shared_ptr<RmagineOptixSpherical>;

// will by generated in cpp
void RegisterRmagineOptixSpherical();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_OPTIX_SPHERICAL_PLUGIN_H