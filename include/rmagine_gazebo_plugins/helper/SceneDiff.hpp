#ifndef RMAGINE_GAZEBO_PLUGINS_SCENE_DIFF_HPP
#define RMAGINE_GAZEBO_PLUGINS_SCENE_DIFF_HPP

#include <unordered_map>
#include <unordered_set>
#include <iostream>

#include <gazebo/physics/Model.hh>

namespace gazebo
{

class SceneDiff
{
public:
    std::unordered_set<uint32_t> added;
    std::unordered_set<uint32_t> removed;

    std::unordered_set<uint32_t> transformed;
    std::unordered_set<uint32_t> scaled;

    std::unordered_map<uint32_t, std::unordered_set<std::string> > joints_changed;


    inline std::unordered_set<uint32_t> changed() const
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

    inline bool ModelJointsChanged() const
    {
        return !joints_changed.empty();
    }

    inline bool ModelChanged() const
    {
        return ModelTransformed() || ModelScaled() || ModelJointsChanged();
    }

    inline bool HasChanged() const
    {
        return ModelAdded() || ModelRemoved() || ModelChanged() || ModelJointsChanged();
    }

    void ComputeAdded(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new);

    void ComputeRemoved(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new);

    void ComputeTransformed(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_map<uint32_t, ignition::math::Pose3d>& poses,
        float change_delta_trans, 
        float change_delta_rot);

    void ComputeScaled(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_map<uint32_t, ignition::math::Vector3d>& scales,
        float change_delta_scale);

    void ComputeJointsChanged(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models,
        const std::unordered_map<uint32_t, 
            std::unordered_map<std::string, ignition::math::Pose3d> 
        >& model_link_poses,
        float change_delta_trans, 
        float change_delta_rot);
};

} // namespace gazebo


inline std::ostream& operator<<(std::ostream& os, const gazebo::SceneDiff& diff)
{
    os << "SceneDiff:\n";
    os << "- added: " << diff.added.size() << "\n";
    os << "- removed: " << diff.removed.size() << "\n";
    os << "- changed: " << diff.changed().size() << "\n";
    os << "-- transformed: " << diff.transformed.size() << "\n";
    os << "-- scaled: " << diff.scaled.size() << "\n";
    os << "-- models of changed joints: " << diff.joints_changed.size() << "\n";
    return os;
}

#endif // RMAGINE_GAZEBO_PLUGINS_SCENE_DIFF_HPP