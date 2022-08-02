#include "rmagine_gazebo_plugins/helper/SceneDiff.hpp"

#include <gazebo/physics/physics.hh>

namespace gazebo
{

void SceneDiff::ComputeAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new)
{
    added.clear();
    for(auto elem : models_new)
    {
        auto it = models_old.find(elem.first);
        if(it == models_old.end())
        {
            // new is not in old -> element was added
            added.insert(elem.first);
        }
    }
}

void SceneDiff::ComputeRemoved(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new)
{
    removed.clear();
    for(auto elem : models_old)
    {
        auto it = models_new.find(elem.first);
        if(it == models_new.end() )
        {
            // old is not in new anymore
            removed.insert(elem.first);
        }
    }
}

void SceneDiff::ComputeTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_map<uint32_t, ignition::math::Pose3d>& poses,
    float change_delta_trans, float change_delta_rot)
{
    transformed.clear();
    for(auto elem : models)
    {
        // 1. check pose change
        auto it_pose_old = poses.find(elem.first);

        if(it_pose_old != poses.end())
        {
            // also exists in old models poses
            // next: check if model pose was changed
            ignition::math::Pose3d pose_new = elem.second->RelativePose();
            ignition::math::Pose3d pose_old = it_pose_old->second;
            ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

            if(pose_delta.Pos().Length() > change_delta_trans)
            {
                // translation delta high enough
                transformed.insert(elem.first);
            } else {
                ignition::math::Quaternion q = pose_delta.Rot();
                ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
                double angle = 2.0 * atan2(vec.Length(), q.W());

                if(angle > change_delta_rot)
                {
                    // angle delta high enough
                    transformed.insert(elem.first);
                }
            }
        }
    }
}

void SceneDiff::ComputeScaled(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_map<uint32_t, ignition::math::Vector3d>& scales,
    float change_delta_scale) 
{
    scaled.clear();
    for(auto elem : models)
    {
        // 2. check scale change
        auto it_scale_old = scales.find(elem.first);

        if(it_scale_old != scales.end())
        {
            double scale_change = (it_scale_old->second - elem.second->Scale()).Length();
            if(scale_change > change_delta_scale)
            {
                scaled.insert(elem.first);
            }
        }
    }
}

void SceneDiff::ComputeJointsChanged(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_map<uint32_t, 
        std::unordered_map<std::string, ignition::math::Pose3d> 
    >& model_link_poses,
    float change_delta_trans, 
    float change_delta_rot) 
{
    joints_changed.clear();
    for(auto elem : models)
    {
        // 2. check scale change
        uint32_t model_id = elem.first;

        auto model_it = model_link_poses.find(elem.first);
        if(model_it != model_link_poses.end())
        {
            // found model in extra joints
            for(auto joint : elem.second->GetJoints())
            {
                physics::LinkPtr child_link = joint->GetJointLink(0);
                if(child_link)
                {
                    auto link_it = model_it->second.find(child_link->GetName());
                    if(link_it != model_it->second.end())
                    {
                        // link found
                        ignition::math::Pose3d pose_new = child_link->RelativePose();
                        ignition::math::Pose3d pose_old = link_it->second;
                        ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

                        if(pose_delta.Pos().Length() > change_delta_trans)
                        {
                            if(joints_changed.find(model_id) == joints_changed.end())
                            {
                                joints_changed[model_id] = {};
                            }
                            joints_changed[model_id].insert(child_link->GetName());
                        } else {
                            ignition::math::Quaternion q = pose_delta.Rot();
                            ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
                            double angle = 2.0 * atan2(vec.Length(), q.W());

                            if(angle > change_delta_rot)
                            {
                                // angle delta high enough
                                if(joints_changed.find(model_id) == joints_changed.end())
                                {
                                    joints_changed[model_id] = {};
                                }
                                joints_changed[model_id].insert(child_link->GetName());
                            }
                        }
                    }
                }
            }
        }
    }
}

} // namespace gazebo