#ifndef GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>


#include <rmagine/noise/Noise.hpp>

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

    // inline rm::MemoryView<float, rm::RAM> ranges() const
    // {
    //     return m_ranges;
    // }

    rm::IntAttrAny<rm::RAM> sim_buffers;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    virtual void Fini() override;

    bool m_needs_update = false;

    rm::SphericalModel m_sensor_model;
    rm::Transform m_Tsb;


    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rm::EmbreeMapPtr m_map;
    rm::SphereSimulatorEmbreePtr m_sphere_sim;

    bool m_gz_publish = false;

    std::vector<rm::NoisePtr> m_noise_models;

    


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