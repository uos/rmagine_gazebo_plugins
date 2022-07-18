#include <rmagine_gazebo_plugins/rmagine_embree_map_gzplugin.h>
#include <iostream>
#include <gazebo/sensors/SensorsIface.hh>


#include <gazebo/sensors/SensorManager.hh>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>

using namespace std::placeholders;
using namespace boost::algorithm;

namespace gazebo
{

RmagineEmbreeMap::RmagineEmbreeMap()
{
    ROS_INFO("Constructing RmagineEmbreeMap.");
}

RmagineEmbreeMap::~RmagineEmbreeMap()
{
    ROS_INFO("Destroying RmagineEmbreeMap.");
}

void RmagineEmbreeMap::Load(
    physics::WorldPtr _world, 
    sdf::ElementPtr _sdf)
{
    m_world = _world;
    m_sdf = _sdf;

    m_world_update_conn= event::Events::ConnectWorldUpdateBegin(
        std::bind(&RmagineEmbreeMap::OnWorldUpdate, this, std::placeholders::_1)
    );

    // create empty map
    m_map.reset(new rmagine::EmbreeMap);

    // connect to simulators
    m_sphere_sim = std::make_shared<rmagine::SphereSimulatorEmbree>(m_map);
}

std::unordered_map<uint32_t, physics::ModelPtr> RmagineEmbreeMap::ToIdMap(
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

std::unordered_set<uint32_t> RmagineEmbreeMap::ComputeAdded(
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

std::unordered_set<uint32_t> RmagineEmbreeMap::ComputeRemoved(
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

std::unordered_set<uint32_t> RmagineEmbreeMap::ComputeChanged(
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

ModelsDiff RmagineEmbreeMap::ComputeDiff(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    ModelsDiff ret;
    ret.added = ComputeAdded(models_old, models_new);
    ret.removed = ComputeRemoved(models_old, models_new);
    ret.changed = ComputeChanged(models_old, models_new);
    return ret;
}

void RmagineEmbreeMap::UpdateState()
{
    std::vector<physics::ModelPtr> models = m_world->Models();

    std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);

    auto diff = ComputeDiff(m_models, models_new);


    // apply changes to rmagine

    if(diff.HasChanged())
    {
        // TODO! translate gazebo models to embree map instances
        
        for(auto model_id : diff.added)
        {

            std::cout << "Add model " << model_id << " to embree map" << std::endl;
        
            physics::ModelPtr model = models_new[model_id];
            
            std::string model_name = model->GetName();

            sdf::ElementPtr sdf = model->GetSDF();

            std::vector<physics::LinkPtr> links = model->GetLinks();

            for(auto link : links)
            {
                std::map<uint32_t, msgs::Visual> visuals = link->Visuals();
                std::cout << "Loaded " << link->GetName() << " -> " << visuals.size() << " visuals" << std::endl;
                for(auto elem : visuals)
                {
                    msgs::Visual vis = elem.second;
                    if(vis.has_geometry())
                    {
                        msgs::Geometry geom = vis.geometry();
                        if(geom.has_mesh())
                        {
                            std::cout << "MESH" << std::endl;
                        }

                        if(geom.has_box())
                        {
                            std::cout << "BOX" << std::endl;
                        }

                        if(geom.has_cylinder())
                        {
                            std::cout << "CYLINDER" << std::endl;
                        }

                        if(geom.has_sphere())
                        {
                            std::cout << "SPHERE" << std::endl;
                        }

                        if(geom.has_plane())
                        {
                            std::cout << "PLANE" << std::endl;
                            msgs::PlaneGeom plane = geom.plane();

                            msgs::Vector2d size = plane.size();
                            msgs::Vector3d normal = plane.normal();

                            rmagine::EmbreeMeshPtr mesh(new rmagine::EmbreeMesh(
                                m_map->device, 4, 2));

                            mesh->vertices[0].x = -size.x();
                            mesh->vertices[0].y = size.y();
                            mesh->vertices[0].z = 0.0;  

                            mesh->vertices[1].x = size.x();
                            mesh->vertices[1].y = size.y();
                            mesh->vertices[1].z = 0.0;
                            
                            mesh->vertices[2].x = size.x();
                            mesh->vertices[2].y = -size.y();
                            mesh->vertices[2].z = 0.0;

                            mesh->vertices[3].x = size.x();
                            mesh->vertices[3].y = -size.y();
                            mesh->vertices[3].z = 0.0;

                            mesh->faces[0].v0 = 1;
                            mesh->faces[0].v1 = 0;
                            mesh->faces[0].v2 = 3;

                            mesh->faces[1].v0 = 3;
                            mesh->faces[1].v1 = 2;
                            mesh->faces[1].v2 = 1;

                            mesh->commit();

                            // transform

                            unsigned int mesh_id = m_map->addMesh(mesh);
                            std::cout << "Plane added to embree with id " << mesh_id << std::endl;
                        }

                        if(geom.has_heightmap())
                        {
                            std::cout << "HEIGHTMAP" << std::endl;
                        }
                    }
                }
            }
        }

        m_map->scene->commit();

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
            auto model = m_models[rem_name];
            if(model->GetSensorCount() > 0)
            {
                // reload sensors
                m_sensors_loaded = false;
            }

            m_models.erase(rem_name);
            m_poses.erase(rem_name);
            m_scales.erase(rem_name);
        }

        // add
        for(auto add_name : diff.added)
        {
            auto model = models_new[add_name];
            if(model->GetSensorCount() > 0)
            {
                // reload sensors
                m_sensors_loaded = false;
            }

            m_models[add_name] = model;
            m_poses[add_name] = model->RelativePose();
            m_scales[add_name] = model->Scale();
        }
    }
}

void RmagineEmbreeMap::UpdateSensors()
{
    // TODO: get all sensors of certain type, simulate sensor data if required   
    if(!m_sensors_loaded)
    {
        m_sphericals.clear();
        m_sphere_sims.clear();

        std::vector<sensors::SensorPtr> sensors 
            = sensors::SensorManager::Instance()->GetSensors();

        for(sensors::SensorPtr sensor : sensors)
        {
            sensors::RmagineEmbreeSphericalPtr spherical 
                = std::dynamic_pointer_cast<sensors::RmagineEmbreeSpherical>(sensor);

            if(spherical)
            {
                std::cout << "[RmagineEmbreeMap] Found Rmagine spherical sensor " << spherical->ScopedName() << std::endl;
                std::cout << "- Parent: " << spherical->ParentName() << std::endl;

                std::cout << "- Existing: " << std::endl;
                for(auto elem : m_models)
                {
                    std::cout << "-- " << elem.second->GetName() << std::endl;
                }


                std::string scoped_name = spherical->ScopedName();

                std::vector<std::string> scope_names;

                typedef split_iterator<std::string::iterator> string_split_iterator;
                for(string_split_iterator It=
                    make_split_iterator(scoped_name, first_finder("::", is_iequal()));
                    It!=string_split_iterator();
                    ++It)
                {
                    scope_names.push_back(boost::copy_range<std::string>(*It) );   
                }
                
                // docs:
                // std::cout << "- World: " << scope_names[0] << std::endl;
                // std::cout << "- Model: " << scope_names[1] << std::endl;
                // std::cout << "- Link: " << scope_names[2] << std::endl;
                // std::cout << "- Name: " << scope_names[3] << std::endl;


                physics::ModelPtr model = m_world->ModelByName(scope_names[1]);

                if(!model)
                {
                    std::cout << "[RmagineEmbreeMap] WARNING: Coud not find model " << scope_names[1] << " to sensor " << spherical->ScopedName() << std::endl; 
                }

                if(m_models.find(model->GetId()) == m_models.end())
                {
                    std::cout << "[RmagineEmbreeMap] WARNING: Coud not find model " << scope_names[1] << " in model map: ";
                    for(auto elem : m_models)
                    {
                        std::cout << elem.second->GetName() << ", ";
                    } 
                    std::cout << std::endl;
                }

                // TODO: for dynamic sensor. check if model is uppest (below world)

                


                rmagine::SphereSimulatorEmbreePtr sphere_sim(new rmagine::SphereSimulatorEmbree);

                sphere_sim->setMap(m_map);

                ignition::math::Pose3d gz_pose_sensor = spherical->Pose();
                rmagine::Transform Tsb;
                Tsb.R.x = gz_pose_sensor.Rot().X();
                Tsb.R.y = gz_pose_sensor.Rot().Y();
                Tsb.R.z = gz_pose_sensor.Rot().Z();
                Tsb.R.w = gz_pose_sensor.Rot().W();
                Tsb.t.x = gz_pose_sensor.Pos().X();
                Tsb.t.y = gz_pose_sensor.Pos().Y();
                Tsb.t.z = gz_pose_sensor.Pos().Z();
                sphere_sim->setTsb(Tsb);


                // TODO real model
                rmagine::SphericalModel sensor_model;
                sensor_model.theta.min = -M_PI;
                sensor_model.theta.inc = 0.4 * M_PI / 180.0;
                sensor_model.theta.size = 900;

                // 3D:
                // sensor_model.phi.min = -15.0 * M_PI / 180.0;
                // sensor_model.phi.inc = 2.0 * M_PI / 180.0;
                // sensor_model.phi.size = 16;
                
                // 2D:
                sensor_model.phi.min = 0.0;
                sensor_model.phi.inc = 1.0;
                sensor_model.phi.size = 1;

                sensor_model.range.min = 0.1;
                sensor_model.range.max = 130.0;

                sphere_sim->setModel(sensor_model);

                // store as member
                m_sphericals[spherical->Name()] = spherical;
                m_sphere_sims[spherical->Name()] = sphere_sim;
                m_sphere_parents[spherical->Name()] = model->GetId();
            }
        }

        m_sensors_loaded = true;
    } 
    
    std::vector<std::string> sphericals_to_update;
    for(auto elem : m_sphericals)
    {
        sensors::RmagineEmbreeSphericalPtr spherical = elem.second;
        if(spherical->needsUpdate())
        {
            // update
            sphericals_to_update.push_back(elem.first);
        }
    }

    for(auto spherical_name : sphericals_to_update)
    {
        sensors::RmagineEmbreeSphericalPtr spherical = m_sphericals[spherical_name];
        rmagine::SphereSimulatorEmbreePtr sim = m_sphere_sims[spherical_name];
        
        
        ignition::math::Pose3d gz_pose_model = m_poses[m_sphere_parents[spherical_name]];

        rmagine::Transform Tbm;
        Tbm.R.x = gz_pose_model.Rot().X();
        Tbm.R.y = gz_pose_model.Rot().Y();
        Tbm.R.z = gz_pose_model.Rot().Z();
        Tbm.R.w = gz_pose_model.Rot().W();
        Tbm.t.x = gz_pose_model.Pos().X();
        Tbm.t.y = gz_pose_model.Pos().Y();
        Tbm.t.z = gz_pose_model.Pos().Z();

        rmagine::Memory<rmagine::Transform, rmagine::RAM> Tbms(1);
        Tbms[0] = Tbm;

        std::cout << "Simulate " << spherical->ScopedName() << std::endl;
        auto ranges = sim->simulateRanges(Tbms);
        std::cout << "done. " << std::endl;


        // check results
        unsigned int valid_pts = 0;
        for(unsigned int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] > 0.1 && ranges[i] < 130.0 )
            {
                valid_pts++;
            }
        }

        std::cout << valid_pts << " valid pts" << std::endl;

        // TODO: update sensor
        spherical->update();

        
    }

}

void RmagineEmbreeMap::OnWorldUpdate(const common::UpdateInfo& info)
{



    // this is called every simulation step
    UpdateState();


    UpdateSensors();


}

GZ_REGISTER_WORLD_PLUGIN(RmagineEmbreeMap)

} // namespace gazebo