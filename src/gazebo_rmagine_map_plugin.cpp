#include <gazebo_rmagine_plugin/gazebo_rmagine_map_plugin.h>
#include <iostream>

using namespace std::placeholders;

namespace gazebo
{

RmagineMap::RmagineMap()
{
    ROS_INFO("Constructing RmagineMap.");
}

RmagineMap::~RmagineMap()
{
    ROS_INFO("Destroying RmagineMap.");
}

void RmagineMap::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    m_world = _world;

    m_world_update_conn= event::Events::ConnectWorldUpdateBegin(
        std::bind(&RmagineMap::OnWorldUpdate, this, std::placeholders::_1)
    );

    // std::cout << "RmagineMap::Load !!!!!" << std::endl;
}

std::unordered_map<uint32_t, physics::ModelPtr> RmagineMap::ToIdMap(
    const std::vector<physics::ModelPtr>& models)
{
    std::unordered_map<uint32_t, physics::ModelPtr> ret;

    for(auto model : models)
    {
        if(model)
        {
            ret[model->GetId()] = model;
        }
    }

    return ret;
}

std::unordered_set<uint32_t> RmagineMap::ComputeAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    for(auto elem : models_new)
    {
        auto it = models_old.find(elem.first);
        if(it == models_old.end())
        {
            // new is not in old -> element was added
            ret.insert(elem.first);
        }
    }

    return ret;
}

std::unordered_set<uint32_t> RmagineMap::ComputeRemoved(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    for(auto elem : models_old)
    {
        auto it = models_new.find(elem.first);
        if(it == models_new.end() )
        {
            // old is not in new anymore
            ret.insert(elem.first);
        }
    }

    return ret;
}

std::unordered_set<uint32_t> RmagineMap::ComputeChanged(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    // TODO

    for(auto elem : models_new)
    {
        // 1. check pose change
        auto it_pose_old = m_poses.find(elem.first);
        bool change_found = false;
        if(it_pose_old != m_poses.end())
        {
            // also exists in old models poses
            // next: check if model pose was changed

            ignition::math::Pose3d pose_new = elem.second->RelativePose();
            ignition::math::Pose3d pose_old = it_pose_old->second;
            ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

            if(pose_delta.Pos().Length() > m_changed_delta_trans)
            {
                change_found = true;
            } else {
                ignition::math::Quaternion q = pose_delta.Rot();
                ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
                double angle = 2.0 * atan2(vec.Length(), q.W());

                if(angle > m_changed_delta_rot)
                {
                    change_found = true;
                }
            }
        }

        if(!change_found)
        {
            // 2. check scale change
            auto it_scale_old = m_scales.find(elem.first);

            if(it_scale_old != m_scales.end())
            {
                double scale_change = (it_scale_old->second - elem.second->Scale()).Length();
                if(scale_change > m_changed_delta_scale)
                {
                    change_found = true;
                }
            }

        }
        
        if(change_found)
        {
            ret.insert(elem.first);
        }
    }

    return ret;
}

ModelsDiff RmagineMap::ComputeDiff(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    ModelsDiff ret;
    ret.added = ComputeAdded(models_old, models_new);
    ret.removed = ComputeRemoved(models_old, models_new);
    ret.changed = ComputeChanged(models_old, models_new);
    return ret;
}

void RmagineMap::UpdateState()
{
    std::vector<physics::ModelPtr> models = m_world->Models();

    std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);

    auto diff = ComputeDiff(m_models, models_new);


    // apply changes to rmagine


    
    if(diff.HasChanged())
    {
        // TODO! translate gazebo models to embree map instances
        
        // 1. change existing meshes in embree map

        // 2. add new meshes

        // 3. remove meshes
        
        // if(!m_e_device)
        // {
        //     m_e_device.reset(new rmagine::EmbreeDevice);
        // }
        // if(!m_map)
        // {
        //     m_map.reset(new rmagine::EmbreeMap(m_e_device));
        // }
    }



    // apply changes to gazebo state
    if(diff.HasChanged())
    {
        std::cout << "Diff: " << std::endl;
        std::cout << "- added: " << diff.added.size() << std::endl;
        std::cout << "- removed: " << diff.removed.size() << std::endl;
        std::cout << "- changed: " << diff.changed.size() << std::endl;

        // change
        for(auto change_name : diff.changed)
        {
            m_poses[change_name] = models_new[change_name]->RelativePose();
            m_scales[change_name] = models_new[change_name]->Scale();
        }

        // remove
        for(auto rem_name : diff.removed)
        {
            m_models.erase(rem_name);
            m_poses.erase(rem_name);
            m_scales.erase(rem_name);
        }

        // add
        for(auto add_name : diff.added)
        {
            m_models[add_name] = models_new[add_name];
            m_poses[add_name] = models_new[add_name]->RelativePose();
            m_scales[add_name] = models_new[add_name]->Scale();
        }
    }
}

void RmagineMap::OnWorldUpdate(const common::UpdateInfo& info)
{
    // this is called every simulation step
    UpdateState();
}

GZ_REGISTER_WORLD_PLUGIN(RmagineMap)

} // namespace gazebo