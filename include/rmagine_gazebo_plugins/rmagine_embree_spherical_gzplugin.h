#ifndef GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H

#include <ros/ros.h>


#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{

namespace sensors
{

class RmagineEmbreeSpherical : public RaySensor
{
public:
    using Base = RaySensor;

    RmagineEmbreeSpherical();

    virtual ~RmagineEmbreeSpherical();

    virtual void Init() override;

    virtual void Load(const std::string& world_name) override;

    void update();

    bool needsUpdate() const;


protected:
    virtual bool UpdateImpl(const bool _force) override;

    bool m_needs_update = false;

// protected:
//     virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

};

using RmagineEmbreeSphericalPtr = std::shared_ptr<RmagineEmbreeSpherical>;

// will by generated in cpp
void RegisterRmagineEmbreeSpherical();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_SPHERICAL_PLUGIN_H