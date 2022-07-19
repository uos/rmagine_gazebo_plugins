#ifndef GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H

#include <ros/ros.h>


#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/noise/noise.h>
#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

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

    void updateScanMsg(rm::MemoryView<float> ranges);

protected:
    virtual bool UpdateImpl(const bool _force) override;

    virtual void Fini() override;

    bool m_needs_update = false;

    rm::SphericalModel m_sensor_model;
    rm::Transform m_Tsb;

    rm::SphereSimulatorEmbreePtr m_sphere_sim;

    bool m_pre_alloc_mem = true;
    rm::Memory<float, rm::RAM> m_ranges;


private:

    /// \brief Parent entity pointer
    physics::EntityPtr parentEntity;

    /// \brief Publisher for the scans
    transport::PublisherPtr scanPub;

    /// \brief Laser message.
    msgs::LaserScanStamped laserMsg;

    /// \brief Mutex to protect laserMsg
    std::mutex mutex;
};

using RmagineEmbreeSphericalPtr = std::shared_ptr<RmagineEmbreeSpherical>;

// will by generated in cpp
void RegisterRmagineEmbreeSpherical();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H