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

    SceneDiff(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new);

private:
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

    std::unordered_map<uint32_t, std::unordered_set<std::string> > ComputeJointsChanged(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const;
};

} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_SCENE_DIFF_HPP