#ifndef GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H
#define GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H

#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

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



// rmagine
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

namespace gazebo
{

template<typename T>
std::unordered_set<T> get_union(
    const std::unordered_set<T>& a, const std::unordered_set<T>& b)
{
    std::unordered_set<T> res = a;
    res.insert(b.begin(), b.end());
    return res;
}

struct ModelsDiff 
{
    std::unordered_set<uint32_t> added;
    std::unordered_set<uint32_t> removed;

    std::unordered_set<uint32_t> transformed;
    std::unordered_set<uint32_t> scaled;

    std::unordered_set<uint32_t> changed() const
    {
        std::unordered_set<uint32_t> res = transformed;
        res.insert(scaled.begin(), scaled.end());
        return res;
    }

    inline bool ModelAdded() const 
    {
        return !added.empty();
    }

    inline bool ModelRemoved() const 
    {
        return !removed.empty();
    }

    inline bool ModelTransformed() const
    {
        return !transformed.empty();
    }

    inline bool ModelScaled() const
    {
        return !scaled.empty();
    }

    inline bool ModelChanged() const
    {
        return ModelTransformed() || ModelScaled();
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

    std::unordered_set<uint32_t> ComputeTransformed(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;

    std::unordered_set<uint32_t> ComputeScaled(
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

    rmagine::EmbreeMeshPtr to_rmagine(const msgs::SphereGeom& sphere) const;

    rmagine::EmbreeMeshPtr to_rmagine(const msgs::CylinderGeom& cylinder) const;

    rmagine::EmbreeMeshPtr to_rmagine(const msgs::MeshGeom& gzmesh) const;

    void UpdateState();

    void UpdateSensors();


    physics::WorldPtr m_world;
    sdf::ElementPtr m_sdf;

    event::ConnectionPtr m_world_update_conn;

    
    std::shared_ptr<std::shared_mutex> m_map_mutex;
    rmagine::EmbreeMapPtr m_map;

    bool m_sensors_loaded = false;

    double m_changed_delta_trans = 0.001; // meter
    double m_changed_delta_rot = 0.001; // radian
    double m_changed_delta_scale = 0.001;

    // TODO: somehow update meshes in embree map
    // model (rel pose change)
    // - link1 (rel pose change?)
    // - link2
    
    
    std::unordered_map<uint32_t, std::vector<rm::EmbreeMeshPtr> > m_model_meshes;
    std::unordered_set<uint32_t> m_model_ignores;
    std::unordered_set<uint32_t> m_link_ignores;
    std::unordered_set<uint32_t> m_visual_ignores;


    std::unordered_map<uint32_t, physics::ModelPtr> m_models;
    std::unordered_map<std::string, unsigned int> m_visual_to_mesh;


    std::unordered_map<uint32_t, ignition::math::Pose3d> m_poses;
    std::unordered_map<uint32_t, ignition::math::Vector3d> m_scales;

    std::future<void> m_updater_thread;

};

} // namespace gazebo

inline std::ostream& operator<<(std::ostream& os, const gazebo::ModelsDiff& diff)
{
    os << "ModelsDiff:\n";
    os << "- added: " << diff.added.size() << "\n";
    os << "- removed: " << diff.removed.size() << "\n";
    os << "- changed: " << diff.changed().size() << "\n";
    os << "-- transformed: " << diff.transformed.size() << "\n";
    os << "-- scaled: " << diff.scaled.size() << "\n";
    return os;
}

// inline std::ostream& operator<<(std::ostream& os, const gazebo::Transform& T)
// {
//     os << "T[" << T.t << ", " << T.R << "]";
//     return os;
// }


#endif // GAZEBO_RMAGINE_EMBREE_MAP_PLUGIN_H