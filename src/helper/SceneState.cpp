#include "rmagine_gazebo_plugins/helper/SceneState.hpp"
#include <iostream>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Console.hh>


namespace gazebo
{

SceneDiff SceneState::diff(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models, 
    float change_delta_trans,
    float change_delta_rot,
    float change_delta_scale) const
{
    SceneDiff ret;

    ret.ComputeAdded(m_models, models);
    ret.ComputeRemoved(m_models, models);
    ret.ComputeTransformed(models, m_poses, change_delta_trans, change_delta_rot);
    ret.ComputeScaled(models, m_scales, change_delta_scale);
    ret.ComputeJointsChanged(models, m_model_link_poses, change_delta_trans, change_delta_rot);

    return ret;
}

bool SceneState::update(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const SceneDiff& diff)
{
    bool reload_sensors = false;

    if(diff.HasChanged())
    {
        // change
        for(auto change_id : diff.changed())
        {
            auto model = m_models[change_id];
            m_poses[change_id] = model->RelativePose();
            m_scales[change_id] = model->Scale();
        }

        // joint changes
        for(auto elem : diff.joints_changed)
        {
            auto model = m_models[elem.first];
            for(std::string link_name : elem.second)
            {
                auto link = model->GetLink(link_name);
                m_model_link_poses[elem.first][link_name] = link->RelativePose();
            }
        }

        // remove
        for(auto rem_id : diff.removed)
        {
            auto model = m_models[rem_id];
            if(model->GetSensorCount() > 0)
            {
                // reload sensors
                reload_sensors = true;
            }

            m_models.erase(rem_id);
            m_poses.erase(rem_id);
            m_scales.erase(rem_id);
            m_model_link_poses.erase(rem_id);
        }

        // add
        for(auto add_id : diff.added)
        {
            auto model = models.at(add_id);
            if(model->GetSensorCount() > 0)
            {
                // reload sensors
                reload_sensors = true;
            }

            m_models[add_id] = model;
            m_poses[add_id] = model->RelativePose();
            m_scales[add_id] = model->Scale();

            // add all joints
            for(auto joint : model->GetJoints())
            {
                physics::LinkPtr child_link = joint->GetJointLink(0);
                if(child_link)
                {
                    m_model_link_poses[add_id][child_link->GetName()] = child_link->RelativePose();
                    gzlog << "[SceneState] Register jointed link " << child_link->GetScopedName() << " for futher update checks" << std::endl;
                } else {
                    gzwarn << "[SceneState] Could not get child link of joint '" << joint->GetScopedName() << "'" << std::endl;
                }
            }
        }
    }

    return reload_sensors;
}

} // namespace gazebo