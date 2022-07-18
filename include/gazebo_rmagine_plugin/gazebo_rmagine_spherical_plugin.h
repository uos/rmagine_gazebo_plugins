#ifndef GAZEBO_RMAGINE_SPHERICAL_PLUGIN_H
#define GAZEBO_RMAGINE_SPHERICAL_PLUGIN_H

#include <ros/ros.h>


#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{

namespace sensors
{

class RmagineSpherical : public RaySensor
{
public:
    using Base = RaySensor;

    RmagineSpherical();

    virtual ~RmagineSpherical();

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

using RmagineSphericalPtr = std::shared_ptr<RmagineSpherical>;

// will by generated in cpp
void RegisterRmagineSpherical();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RMAGINE_SPHERICAL_PLUGIN_H