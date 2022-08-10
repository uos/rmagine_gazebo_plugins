#ifndef GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/noise/noise.h>
#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>

namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RmagineEmbreeSpherical : public Sensor
{
public:
    using Base = Sensor;

    RmagineEmbreeSpherical();

    virtual ~RmagineEmbreeSpherical();

    virtual void Load(const std::string& world_name) override;

    virtual void Init() override;
    
    virtual std::string Topic() const override;

    virtual bool IsActive() const override;

    void setMap(rm::EmbreeMapPtr map);

    void setLock(std::shared_ptr<std::shared_mutex> mutex);

    void updateScanMsg(rm::MemoryView<float> ranges);

    inline common::Time stamp() const 
    {
        return lastMeasurementTime;
    }

    inline rm::SphericalModel sensorModel() const
    {
        return m_sensor_model;
    }
    
    inline rm::Memory<float, rm::RAM> ranges() const
    {
        return m_ranges;
    }

protected:
    virtual bool UpdateImpl(const bool _force) override;

    virtual void Fini() override;

    bool m_needs_update = false;

    rm::SphericalModel m_sensor_model;
    rm::Transform m_Tsb;


    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rm::EmbreeMapPtr m_map;
    rm::SphereSimulatorEmbreePtr m_sphere_sim;

    bool m_pre_alloc_mem = true;
    rm::Memory<float, rm::RAM> m_ranges;

    bool m_gz_publish = false;

    enum NoiseType {
        NONE = 0,
        GAUSSIAN = 1
    };

    NoiseType m_noise_type = NoiseType::NONE;
    float m_noise_stddev = 0.0;
    float m_noise_mean = 0.0;

private:

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

using RmagineEmbreeSphericalPtr = std::shared_ptr<RmagineEmbreeSpherical>;

// will by generated in cpp
void RegisterRmagineEmbreeSpherical();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H