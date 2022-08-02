#include "rmagine_gazebo_plugins/helper/SceneDiff.hpp"

namespace gazebo
{

SceneDiff::SceneDiff(
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
        const std::unordered_map<uint32_t, physics::ModelPtr>& models_new)
{
    added = ComputeAdded(models_old, models_new);
    removed = ComputeRemoved(models_old, models_new);
    transformed = ComputeTransformed(models_old, models_new);
    scaled = ComputeScaled(models_old, models_new);
    joints_changed = ComputeJointsChanged(models_old, models_new);
}

std::unordered_set<uint32_t> SceneDiff::ComputeAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    // for(auto elem : models_new)
    // {
    //     auto it = models_old.find(elem.first);
    //     if(it == models_old.end())
    //     {
    //         // new is not in old -> element was added
    //         ret.insert(elem.first);
    //     }
    // }

    return ret;
}

std::unordered_set<uint32_t> SceneDiff::ComputeRemoved(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    // for(auto elem : models_old)
    // {
    //     auto it = models_new.find(elem.first);
    //     if(it == models_new.end() )
    //     {
    //         // old is not in new anymore
    //         ret.insert(elem.first);
    //     }
    // }

    return ret;
}

std::unordered_set<uint32_t> SceneDiff::ComputeTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    // for(auto elem : models_new)
    // {
    //     // 1. check pose change
    //     auto it_pose_old = m_poses.find(elem.first);

    //     if(it_pose_old != m_poses.end())
    //     {
    //         // also exists in old models poses
    //         // next: check if model pose was changed
    //         ignition::math::Pose3d pose_new = elem.second->RelativePose();
    //         ignition::math::Pose3d pose_old = it_pose_old->second;
    //         ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

    //         if(pose_delta.Pos().Length() > m_changed_delta_trans)
    //         {
    //             // translation delta high enough
    //             ret.insert(elem.first);
    //         } else {
    //             ignition::math::Quaternion q = pose_delta.Rot();
    //             ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
    //             double angle = 2.0 * atan2(vec.Length(), q.W());

    //             if(angle > m_changed_delta_rot)
    //             {
    //                 // angle delta high enough
    //                 ret.insert(elem.first);
    //             }
    //         }
    //     }
    // }

    return ret;
}

std::unordered_set<uint32_t> SceneDiff::ComputeScaled(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    // for(auto elem : models_new)
    // {
    //     // 2. check scale change
    //     auto it_scale_old = m_scales.find(elem.first);

    //     if(it_scale_old != m_scales.end())
    //     {
    //         double scale_change = (it_scale_old->second - elem.second->Scale()).Length();
    //         if(scale_change > m_changed_delta_scale)
    //         {
    //             ret.insert(elem.first);
    //         }
    //     }
    // }

    return ret;
}

std::unordered_map<uint32_t, std::unordered_set<std::string> > SceneDiff::ComputeJointsChanged(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_map<uint32_t, std::unordered_set<std::string> > ret;

    // for(auto elem : models_new)
    // {
    //     // 2. check scale change
    //     uint32_t model_id = elem.first;

    //     auto model_it = m_model_link_poses.find(elem.first);
    //     if(model_it != m_model_link_poses.end())
    //     {
    //         // found model in extra joints
    //         for(auto joint : elem.second->GetJoints())
    //         {
    //             physics::LinkPtr child_link = joint->GetJointLink(0);
    //             if(child_link)
    //             {
    //                 auto link_it = model_it->second.find(child_link->GetName());
    //                 if(link_it != model_it->second.end())
    //                 {
    //                     // link found
    //                     ignition::math::Pose3d pose_new = child_link->RelativePose();
    //                     ignition::math::Pose3d pose_old = link_it->second;
    //                     ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

    //                     if(pose_delta.Pos().Length() > m_changed_delta_trans)
    //                     {
    //                         if(ret.find(model_id) == ret.end())
    //                         {
    //                             ret[model_id] = {};
    //                         }
    //                         ret[model_id].insert(child_link->GetName());
    //                     } else {
    //                         ignition::math::Quaternion q = pose_delta.Rot();
    //                         ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
    //                         double angle = 2.0 * atan2(vec.Length(), q.W());

    //                         if(angle > m_changed_delta_rot)
    //                         {
    //                             // angle delta high enough
    //                             if(ret.find(model_id) == ret.end())
    //                             {
    //                                 ret[model_id] = {};
    //                             }
    //                             ret[model_id].insert(child_link->GetName());
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    return ret;
}

} // namespace gazebo