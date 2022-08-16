#ifndef GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>
#include <rmagine_gazebo_plugins/helper/helper_functions.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


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
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

#include <rmagine/util/StopWatch.hpp>

namespace gazebo
{
struct VisualTransform 
{
    std::string name;
    rmagine::Transform T;
    uint32_t model_id;
};

class RmagineEmbreeMap : public WorldPlugin
{
public: 
    RmagineEmbreeMap();
    virtual ~RmagineEmbreeMap();

protected:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
private:
    void parseParams(sdf::ElementPtr sdf);

    std::unordered_map<rm::EmbreeGeometryPtr, VisualTransform> EmbreeUpdateAdded(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& added);

    std::unordered_set<rm::EmbreeGeometryPtr> EmbreeUpdateTransformed(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& transformed);

    std::unordered_set<rm::EmbreeGeometryPtr> EmbreeUpdateScaled(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_set<uint32_t>& scaled);

    std::unordered_set<rm::EmbreeGeometryPtr> EmbreeUpdateJointChanges(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_map<uint32_t, std::unordered_set<std::string> >& joints_changed);


    void UpdateState(const std::unordered_map<uint32_t, physics::ModelPtr>& models_new,
        const SceneDiff& diff);

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
    rmagine::EmbreeMapPtr m_map;

    enum GeomCacheID
    {
        PLANE = 0,
        BOX = 1,
        SPHERE = 2,
        CYLINDER = 3
    };

    // mesh cache
    std::unordered_map<GeomCacheID, rmagine::EmbreeScenePtr> m_geom_cache;
    std::unordered_map<std::string, rmagine::EmbreeScenePtr> m_mesh_cache;

    bool m_sensors_loaded = false;

    rmagine::StopWatch m_sw_world_update;
    double m_world_update_freq = 0.0;

    // TODO: somehow update meshes in embree map
    // model (rel pose change)
    // - link1 (rel pose change?)
    // - link2
    
    // ignores 
    // - filled by EmbreeUpdateAdded
    // - removed in  UpdateState in remove part
    std::unordered_set<uint32_t> m_model_ignores;
    std::unordered_set<uint32_t> m_model_has_link_ignores;
    std::unordered_set<std::string> m_link_ignores;
    

    std::unordered_map<uint32_t, std::vector<rm::EmbreeGeometryPtr> > m_model_meshes;
    std::unordered_map<std::string, std::vector<rm::EmbreeGeometryPtr> > m_visual_to_geoms;
    std::unordered_map<rm::EmbreeGeometryPtr, VisualTransform> m_geom_to_visual;

    SceneState m_scene_state;

    
    std::thread m_updater_thread;
    bool m_stop_updater_thread = false;
};

} // namespace gazebo


#endif // GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H