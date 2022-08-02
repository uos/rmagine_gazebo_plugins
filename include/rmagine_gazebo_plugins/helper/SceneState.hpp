#ifndef RMAGINE_GAZEBO_PLUGINS_SCENE_STATE_HPP
#define RMAGINE_GAZEBO_PLUGINS_SCENE_STATE_HPP

#include <unordered_map>
#include <unordered_set>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <memory>

#include "SceneDiff.hpp"


namespace gazebo
{

class SceneState
{
public:
    SceneState(const std::unordered_map<uint32_t, physics::ModelPtr>& models);
    
    SceneDiff changes() const;

// TODO private
// private:
    std::unordered_map<uint32_t, physics::ModelPtr> m_models;
    std::unordered_map<uint32_t, ignition::math::Pose3d> m_poses;
    std::unordered_map<uint32_t, ignition::math::Vector3d> m_scales;

    // EXTRA CHECK FOR JOINTS
    // for links of joints
    // map_id -> link_name -> pose
    std::unordered_map<uint32_t, 
        std::unordered_map<std::string, ignition::math::Pose3d> 
    > m_model_link_poses;
};

using SceneStatePtr = std::shared_ptr<SceneState>;


} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_SCENE_STATE_HPP