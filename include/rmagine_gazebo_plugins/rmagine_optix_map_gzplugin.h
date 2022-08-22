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
#include <rmagine/util/StopWatch.hpp>


namespace gazebo
{

struct VisualTransform 
{
    std::string name;
    rmagine::Transform T;
    uint32_t model_id;
};

class RmagineOptixMap : public WorldPlugin
{
public: 
    RmagineOptixMap();
    virtual ~RmagineOptixMap();
protected:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
private:
    void parseParams(sdf::ElementPtr sdf);

    std::unordered_map<rmagine::OptixInstPtr, VisualTransform> OptixUpdateAdded(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& added);

    std::unordered_set<rmagine::OptixInstPtr> OptixUpdateTransformed(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& transformed);

    std::unordered_set<rmagine::OptixInstPtr> OptixUpdateScaled(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& scaled);

    std::unordered_set<rmagine::OptixInstPtr> OptixUpdateJointChanges(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_map<uint32_t, std::unordered_set<std::string> >& joints_changed);

    void UpdateState();

    void UpdateSensors();

    physics::WorldPtr m_world;
    sdf::ElementPtr m_sdf;


    // params changed by sdf
    enum MeshLoading {
        INTERNAL = 0,
        GAZEBO = 1
    };

    double m_changed_delta_trans = 0.001; // meter
    double m_changed_delta_rot = 0.001; // radian
    double m_changed_delta_scale = 0.001;
    double m_update_rate_limit = 200.0;

    std::vector<MeshLoading> m_mesh_loader = {GAZEBO, INTERNAL};
    

    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rmagine::OptixMapPtr m_map;


    enum GeomCacheID
    {
        PLANE = 0,
        BOX = 1,
        SPHERE = 2,
        CYLINDER = 3
    };

    // mesh cache
    std::unordered_map<std::string, rmagine::OptixScenePtr> m_mesh_cache;
    std::unordered_map<GeomCacheID, rmagine::OptixScenePtr> m_geom_cache;
    

    bool m_sensors_loaded = false;


    std::unordered_set<uint32_t> m_model_ignores;
    std::unordered_set<uint32_t> m_model_has_link_ignores;
    std::unordered_set<std::string> m_link_ignores;


    std::unordered_map<uint32_t, std::vector<rmagine::OptixInstPtr> > m_model_meshes;
    std::unordered_map<std::string, std::vector<rmagine::OptixInstPtr> > m_visual_to_geoms;
    std::unordered_map<rmagine::OptixInstPtr, VisualTransform> m_geom_to_visual;


    SceneState m_scene_state;


    std::thread m_updater_thread;
    bool m_stop_updater_thread = false;
};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_OPTIX_MAP_PLUGIN_H