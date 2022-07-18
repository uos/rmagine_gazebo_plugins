#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>

using namespace std::placeholders;

namespace gazebo
{

namespace sensors
{


RmagineEmbreeSpherical::RmagineEmbreeSpherical()
:Base()
{
    std::cout << "[RmagineEmbreeSpherical] Construct !!!!!!!!!!!" << std::endl;
}

RmagineEmbreeSpherical::~RmagineEmbreeSpherical()
{
    std::cout << "[RmagineEmbreeSpherical] Destroy !!!!!!!!!!!" << std::endl;
}

void RmagineEmbreeSpherical::Init()
{
    Base::Init();
    std::cout << "[RmagineEmbreeSpherical] Init !!!!!!!!!!!" << std::endl;
}

void RmagineEmbreeSpherical::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "[RmagineEmbreeSpherical] Load !!!!!!!!!!!" << std::endl;

    std::cout << Base::AngleMin() << std::endl;

}


bool RmagineEmbreeSpherical::UpdateImpl(const bool _force)
{
    bool status = Base::UpdateImpl(_force);
    std::cout << "[RmagineEmbreeSpherical] Update !!!!!!!!!!!" << std::endl;
    m_needs_update = true;
    return status;
}

void RmagineEmbreeSpherical::update()
{
    m_needs_update = false;
}

bool RmagineEmbreeSpherical::needsUpdate() const
{
    return m_needs_update;
}

GZ_REGISTER_STATIC_SENSOR("rmagine_embree_spherical", RmagineEmbreeSpherical)

} // namespace sensors

} // namespace gazebo