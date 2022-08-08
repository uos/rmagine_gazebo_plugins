#ifndef GAZEBO_RMAGINE_OPTIX_MAP_PLUGIN_H
#define GAZEBO_RMAGINE_OPTIX_MAP_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <rmagine_gazebo_plugins/helper/helper_functions.h>


#include <iostream>

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <memory>

#include <future>
#include <thread>
#include <chrono>
#include <shared_mutex>

#include "helper/SceneState.hpp"

// rmagine
#include <rmagine/map/OptixMap.hpp>

namespace gazebo
{

class RmagineOptixMap : public WorldPlugin
{
public: 
    RmagineOptixMap();
    virtual ~RmagineOptixMap();

protected:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    
    void OnWorldUpdate(const common::UpdateInfo& info);
private:

    void UpdateState();

    void UpdateSensors();

    physics::WorldPtr m_world;
    sdf::ElementPtr m_sdf;
    event::ConnectionPtr m_world_update_conn;

    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rmagine::OptixMapPtr m_map;
    

    bool m_sensors_loaded = false;

    double m_changed_delta_trans = 0.001; // meter
    double m_changed_delta_rot = 0.001; // radian
    double m_changed_delta_scale = 0.001;

    std::unordered_set<uint32_t> m_model_ignores;
    std::unordered_set<uint32_t> m_link_ignores;
    std::unordered_set<uint32_t> m_visual_ignores;

    SceneState m_scene_state;

    std::future<void> m_updater_thread;
};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_OPTIX_MAP_PLUGIN_H