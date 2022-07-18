#include <gazebo_rmagine_plugin/gazebo_rmagine_spherical_plugin.h>
#include <iostream>
#include <gazebo/sensors/SensorFactory.hh>
#include <gazebo/physics/World.hh>
// #include <gazebo/physics/WorldPrivate.hh>
#include <gazebo_rmagine_plugin/WorldPrivate.h>

using namespace std::placeholders;

namespace gazebo
{

namespace physics 
{

// class WorldPrivate;

// hack the world
struct WorldHack : World
{
public:
    // using World::dataPtr;
    std::unique_ptr<WorldPrivate> dataPtr;
};

using WorldHackPtr = boost::shared_ptr<WorldHack>;

} // namespace physics

namespace sensors
{


RmagineSpherical::RmagineSpherical()
:Base()
{
    std::cout << "[RmagineSpherical] Construct !!!!!!!!!!!" << std::endl;
}

RmagineSpherical::~RmagineSpherical()
{
    std::cout << "[RmagineSpherical] Destroy !!!!!!!!!!!" << std::endl;
}

void RmagineSpherical::Init()
{
    Base::Init();
    std::cout << "[RmagineSpherical] Init !!!!!!!!!!!" << std::endl;
}

template <typename To, typename From>
inline std::shared_ptr<To> reinterpret_pointer_cast(
    std::shared_ptr<From> const & ptr) noexcept
{ 
    return std::shared_ptr<To>(ptr, reinterpret_cast<To *>(ptr.get())); 
}


void RmagineSpherical::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "[RmagineSpherical] Load !!!!!!!!!!!" << std::endl;

    std::cout << Base::AngleMin() << std::endl;

    // std::cout << "Try to hack world" << std::endl;
    // physics::WorldHack* hack = reinterpret_cast<physics::WorldHack*>(world.get());
    // std::cout << "done." << std::endl;

    // std::cout << "Try to get plugins" << std::endl;
    // // const auto& dataPtr = hack->dataPtr;

    // if(hack->dataPtr)
    // {
    //     auto dataPtr = hack->dataPtr.get();

    //     std::vector<WorldPluginPtr> plugins = dataPtr->plugins;

    //     std::cout << "done." << std::endl;
        
    //     std::cout << "size: " << plugins.size() << std::endl;

        
    // } else {
    //     std::cout << "Could not access private data" << std::endl;
    // }
    


    // const auto& plugins = dataPtr->plugins;
    // const std::vector<physics::WorldPluginPtr>& plugins = dataPtr->plugins;
    
    
    // std::cout << "Num World Plugins: " << hack->numPlugins() << std::endl;
    

}


bool RmagineSpherical::UpdateImpl(const bool _force)
{
    bool status = Base::UpdateImpl(_force);
    std::cout << "[RmagineSpherical] Update !!!!!!!!!!!" << std::endl;
    m_needs_update = true;
    return status;
}

void RmagineSpherical::update()
{
    m_needs_update = false;
}

bool RmagineSpherical::needsUpdate() const
{
    return m_needs_update;
}

GZ_REGISTER_STATIC_SENSOR("rmagine_spherical", RmagineSpherical)

} // namespace sensors

} // namespace gazebo