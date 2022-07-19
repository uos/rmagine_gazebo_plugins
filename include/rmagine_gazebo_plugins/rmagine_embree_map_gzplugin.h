#ifndef GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <memory>



// rmagine
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

namespace gazebo
{

struct ModelsDiff 
{
    std::unordered_set<uint32_t> added;
    std::unordered_set<uint32_t> removed;
    std::unordered_set<uint32_t> changed;

    inline bool ModelAdded() const 
    {
        return !added.empty();
    }

    inline bool ModelRemoved() const 
    {
        return !removed.empty();
    }

    inline bool ModelChanged() const
    {
        return !changed.empty();
    }

    inline bool HasChanged() const
    {
        return ModelAdded() || ModelRemoved() || ModelChanged();
    } 

};

class RmagineEmbreeMap : public WorldPlugin
{
public: 
    RmagineEmbreeMap();
    virtual ~RmagineEmbreeMap();

protected:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    
    void OnWorldUpdate(const common::UpdateInfo& info);

private:

    // todo static
    std::unordered_map<uint32_t, physics::ModelPtr> ToIdMap(
        const std::vector<physics::ModelPtr>& models);

    std::unordered_set<uint32_t> ComputeAdded(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;

    std::unordered_set<uint32_t> ComputeRemoved(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;

    std::unordered_set<uint32_t> ComputeChanged(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;

    ModelsDiff ComputeDiff(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;


    rmagine::EmbreeMeshPtr to_rmagine(const msgs::PlaneGeom& plane) const;

    rmagine::EmbreeMeshPtr to_rmagine(const msgs::BoxGeom& box) const;


    void UpdateState();

    void UpdateSensors();

    physics::WorldPtr m_world;
    sdf::ElementPtr m_sdf;

    event::ConnectionPtr m_world_update_conn;

    // rmagine::EmbreeDevicePtr m_e_device;
    rmagine::EmbreeMapPtr m_map;

    // rmagine::SphereSimulatorEmbreePtr m_sphere_sim;


    // std::unordered_map<uint32_t, rmagine::EmbreeMesh> m_rm_meshes;

    bool m_sensors_loaded = false;
    std::vector<std::string> m_sensors;
    std::unordered_map<std::string, sensors::RmagineEmbreeSphericalPtr> m_sphericals;
    // std::unordered_map<std::string, rmagine::SphereSimulatorEmbreePtr> m_sphere_sims;
    std::unordered_map<std::string, uint32_t> m_sphere_parents;
    


    double m_changed_delta_trans = 0.00001;
    double m_changed_delta_rot = 0.001;
    double m_changed_delta_scale = 0.00001;

    
    std::unordered_map<uint32_t, physics::ModelPtr> m_models;

    std::unordered_map<uint32_t, ignition::math::Pose3d> m_poses;
    std::unordered_map<uint32_t, ignition::math::Vector3d> m_scales;

};

} // namespace gazebo

#endif // GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H