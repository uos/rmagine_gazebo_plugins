#include <rmagine_gazebo_plugins/rmagine_embree_map_gzplugin.h>

#include <rmagine_gazebo_plugins/helper/conversions.h>
#include <rmagine_gazebo_plugins/helper/embree_conversions.h>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>


#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/HeightmapData.hh>
#include <gazebo/common/Console.hh>


#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <rmagine/util/synthetic.h>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/embree/embree_shapes.h>


#include <iostream>
#include <chrono>

using namespace std::placeholders;
using namespace boost::algorithm;
using namespace std::chrono_literals;


namespace rm = rmagine;

namespace gazebo
{

RmagineEmbreeMap::RmagineEmbreeMap()
{
    std::cout << "[RmagineEmbreeMap] Constructed." << std::endl;
}

RmagineEmbreeMap::~RmagineEmbreeMap()
{
    std::cout << "[RmagineEmbreeMap] Destroyed." << std::endl;
}

void RmagineEmbreeMap::Load(
    physics::WorldPtr _world, 
    sdf::ElementPtr _sdf)
{
    m_map_mutex = std::make_shared<std::shared_mutex>();
    m_world = _world;
    m_sdf = _sdf;

    m_world_update_conn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RmagineEmbreeMap::OnWorldUpdate, this, std::placeholders::_1)
    );
    
    // create empty map
    m_map = std::make_shared<rm::EmbreeMap>();
    
    // For gazebo dynamic environments
    m_map->scene->setQuality(RTC_BUILD_QUALITY_LOW);
    m_map->scene->setFlags(RTC_SCENE_FLAG_DYNAMIC);

    std::cout << "[RmagineEmbreeMap] Loaded." << std::endl;
}

std::unordered_map<rm::EmbreeGeometryPtr, VisualTransform> RmagineEmbreeMap::EmbreeUpdateAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& added) const
{
    std::unordered_map<rm::EmbreeGeometryPtr, VisualTransform> geom_to_visual;

    for(auto model_id : added)
    {
        if(m_model_ignores.find(model_id) != m_model_ignores.end())
        {
            continue;
        }

        auto model_it = models.find(model_id);
        if(model_it == models.end())
        {
            continue;
        }
        physics::ModelPtr model = model_it->second;

        std::string model_name = model->GetName();
        
        std::vector<physics::LinkPtr> links = model->GetLinks();
        for(physics::LinkPtr link : links)
        {
            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

            ignition::math::Pose3d link_world_pose = link->WorldPose();
            rm::Transform Tlw = to_rm(link_world_pose);
            
            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                msgs::Vector3d vis_scale = vis.scale();
                msgs::Pose vis_pose = vis.pose();

                rm::Vector Svl = to_rm(vis_scale);

                if(Svl.x > 1.001 || Svl.y > 1.001 || Svl.z > 1.001
                    || Svl.x < 0.999 || Svl.y < 0.999 || Svl.z < 0.999)
                {
                    std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] WARNING: Scale from visual to link currently unused but it seems to be set to " << Svl << std::endl; 
                }

                rm::Transform Tvl = to_rm(vis_pose);
                rm::Transform Tvw = Tlw * Tvl;
                std::string key = vis.name();

                if(vis.has_geometry())
                {
                    msgs::Geometry gzgeom = vis.geometry();

                    std::vector<rm::EmbreeGeometryPtr> geoms;
                    std::unordered_set<rm::EmbreeGeometryPtr> geoms_ignore_model_transform;

                    if(gzgeom.has_box())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD BOX!" << std::endl;
                        msgs::BoxGeom box = gzgeom.box();
                        geoms.push_back(to_rm_embree(box));
                    }

                    if(gzgeom.has_cylinder())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD CYLINDER!" << std::endl;
                        msgs::CylinderGeom cylinder = gzgeom.cylinder();
                        geoms.push_back(to_rm_embree(cylinder));
                    }

                    if(gzgeom.has_sphere())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD SPHERE!" << std::endl;
                        msgs::SphereGeom sphere = gzgeom.sphere();
                        geoms.push_back(to_rm_embree(sphere));
                    }

                    if(gzgeom.has_plane())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD PLANE!" << std::endl;
                        msgs::PlaneGeom plane = gzgeom.plane();
                        geoms.push_back(to_rm_embree(plane));
                    }

                    if(gzgeom.has_heightmap())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD HEIGHTMAP!" << std::endl;
                        // std::cout << "- " << gzgeom.

                        rm::EmbreeGeometryPtr geom = to_rm_embree(gzgeom.heightmap());
                        if(geom)
                        {
                            geoms.push_back(geom);
                            geoms_ignore_model_transform.insert(geom);
                        }
                    }

                    if(gzgeom.has_mesh())
                    {
                        std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] ADD MESH..." << std::endl;
                        msgs::MeshGeom gzmesh = gzgeom.mesh();

                        rm::EmbreeScenePtr mesh_scene;

                        for(auto loader_it = m_mesh_loader.begin(); loader_it != m_mesh_loader.end() && !mesh_scene; ++loader_it)
                        {
                            if(*loader_it == MeshLoading::INTERNAL)
                            {
                                std::cout << "[RmagineEmbreeMap] Assimp mesh loading" << std::endl;
                                mesh_scene = to_rm_embree_assimp(gzmesh);
                            } else if(*loader_it == MeshLoading::GAZEBO) {
                                std::cout << "[RmagineEmbreeMap] Gazebo mesh loading" << std::endl;
                                mesh_scene = to_rm_embree_gazebo(gzmesh);
                                std::cout << "[RmagineEmbreeMap] Gazebo mesh loading done." <<  std::endl;
                            }
                        }

                        if(mesh_scene)
                        {
                            // rm::EmbreeScenePtr mesh_scene = to_rm_embree(gzmesh);
                            std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] - sub instances: " << mesh_scene->count<rm::EmbreeInstance>() << std::endl;
                            std::cout << "[RmagineEmbreeMap - EmbreeUpdateAdded()] - sub meshes: " << mesh_scene->count<rm::EmbreeMesh>() << std::endl;
                            
                            for(auto elem : mesh_scene->geometries())
                            {
                                geoms.push_back(elem.second);
                            }
                        } else {
                            std::cout << "[RmagineEmbreeMap] WARNING add mesh failed. Could not load " << gzmesh.filename() << std::endl;
                        }
                    }

                    for(auto geom : geoms)
                    {
                        // Transform from instance to visual (or is it to world: TODO check)
                        auto Tiv = geom->transform();
                        

                        // Set transform from instance to world
                        if(geoms_ignore_model_transform.find(geom) == geoms_ignore_model_transform.end())
                        {
                            auto Tiw = Tvw * Tiv;
                            geom->setTransform(Tiw);
                        }
                        
                        geom->apply();
                        geom->commit();

                        geom_to_visual[geom] = {key, Tiv, model_id};
                    }
                }
            }
        }
    }

    return geom_to_visual;
}

std::unordered_set<rm::EmbreeGeometryPtr> RmagineEmbreeMap::EmbreeUpdateTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& transformed)
{
    std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_transform;

    for(auto model_id : transformed)
    {
        if(m_model_ignores.find(model_id) != m_model_ignores.end())
        {
            continue;
        }

        // get embree id of model
        auto model_it = models.find(model_id);
        if(model_it == models.end())
        {
            continue;
        }
        physics::ModelPtr model = model_it->second;

        std::string model_name = model->GetName();
        std::vector<physics::LinkPtr> links = model->GetLinks();

        ignition::math::Vector3d model_scale_gz = model->Scale();
        rm::Vector3 model_scale = to_rm(model_scale_gz);

        for(physics::LinkPtr link : links)
        {
            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                std::string key = vis.name();

                auto mesh_vis_it = m_visual_to_geoms.find(key);
                if(mesh_vis_it == m_visual_to_geoms.end())
                {
                    std::cout << "WARNING mesh to update not found in embree. Skipping." << std::endl;
                    std::cout << "- key: " << key << std::endl;
                    continue;
                }

                for(auto geom : mesh_vis_it->second)
                {
                    auto geom_id_opt = m_map->scene->getOpt(geom);
                    if(geom_id_opt)
                    {
                        // exists in scene
                        unsigned int geom_id = *geom_id_opt;
                        // std::cout << "Found visual to transform" << std::endl;
                        // std::cout << "- key: " << key << std::endl;
                        // std::cout << "- id: " << geom_id << std::endl;

                        ignition::math::Pose3d link_world_pose = link->WorldPose();
                        msgs::Pose vis_pose = vis.pose();

                        // convert to rmagine
                        rm::Transform Tlw = to_rm(link_world_pose);
                        rm::Transform Tvl = to_rm(vis_pose);

                        rm::Transform Tvw = Tlw * Tvl;
                        // std::cout << "- transform: " << Tvw << std::endl;

                        auto Tiv = m_geom_to_visual[geom].T;
                        geom->setTransform(Tvw * Tiv);
                        meshes_to_transform.insert(geom);

                    } else {
                        std::cout << "WARNING: visual not in mesh set. But it should." << std::endl;
                        std::cout << "- key: " << key << std::endl;
                    }
                }
            }
        }
    }

    return meshes_to_transform;
}

std::unordered_set<rm::EmbreeGeometryPtr> RmagineEmbreeMap::EmbreeUpdateScaled(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& scaled)
{
    std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_scale;
    for(auto model_id : scaled)
    {
        if(m_model_ignores.find(model_id) != m_model_ignores.end())
        {
            continue;
        }

        // get embree id of model
        auto model_it = models.find(model_id);
        if(model_it == models.end())
        {
            continue;
        }
        physics::ModelPtr model = model_it->second;

        std::string model_name = model->GetName();
        std::vector<physics::LinkPtr> links = model->GetLinks();

        ignition::math::Vector3d model_scale_gz = model->Scale();
        rm::Vector3 model_scale = to_rm(model_scale_gz);

        for(physics::LinkPtr link : links)
        {
            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                std::string key = vis.name();
                
                auto mesh_vis_it = m_visual_to_geoms.find(key);
                if(mesh_vis_it == m_visual_to_geoms.end())
                {
                    std::cout << "WARNING mesh to update not found in embree. Skipping." << std::endl;
                    std::cout << "- key: " << key << std::endl;
                    continue;
                }

                auto geoms = mesh_vis_it->second;

                for(auto geom : geoms)
                {
                    auto geom_id_opt = m_map->scene->getOpt(geom);
                    if(geom_id_opt)
                    {
                        // exists in scene
                        unsigned int geom_id = *geom_id_opt;
                        // std::cout << "Found visual to scale" << std::endl;
                        // std::cout << "- key: " << key << std::endl;
                        // std::cout << "- id: " << geom_id << std::endl;

                        msgs::Vector3d vis_scale = vis.scale();

                        // convert to rmagine
                        rm::Vector Svl = to_rm(vis_scale);

                        // std::cout << "- mesh scale old: " << geom->scale() << std::endl;
                        // std::cout << "- model scale: " << model_scale << std::endl;
                        // std::cout << "- visual scale: " << Svl << std::endl;
                        // std::cout << "- transform: " << Tvw << std::endl;

                        geom->setScale(model_scale);

                        meshes_to_scale.insert(geom);
                    } else {
                        std::cout << "[RmagineEmbreeMap] WARNING mesh seems to be lost somewhere." << std::endl; 
                        std::cout << "[RmagineEmbreeMap] - key: " << key << std::endl;
                    }
                }
            }
        }
    }

    return meshes_to_scale;
}

std::unordered_set<rm::EmbreeGeometryPtr> RmagineEmbreeMap::EmbreeUpdateJointChanges(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_map<uint32_t, std::unordered_set<std::string> >& joints_changed)
{
    std::unordered_set<rm::EmbreeGeometryPtr> mesh_links_to_update;

    for(auto elem : joints_changed)
    {
        uint32_t model_id = elem.first;
        if(m_model_ignores.find(model_id) != m_model_ignores.end())
        {
            continue;
        }

        // get embree id of model
        auto model_it = models.find(model_id);
        if(model_it == models.end())
        {
            continue;
        }
        physics::ModelPtr model = model_it->second;

        for(std::string link_name : elem.second)
        {
            physics::LinkPtr link = model->GetLink(link_name);
            if(link)
            {
                std::map<uint32_t, msgs::Visual> visuals = link->Visuals();
                for(auto elem : visuals)
                {
                    msgs::Visual vis = elem.second;
                    std::string key = vis.name();

                    auto mesh_vis_it = m_visual_to_geoms.find(key);
                    if(mesh_vis_it == m_visual_to_geoms.end())
                    {
                        std::cout << "[RmagineEmbreeMap] WARNING mesh to update not found in embree. Skipping." << std::endl;
                        std::cout << "[RmagineEmbreeMap] - key: " << key << std::endl;
                        continue;
                    }

                    for(auto geom : mesh_vis_it->second)
                    {
                        auto geom_id_opt = m_map->scene->getOpt(geom);
                        if(geom_id_opt)
                        {
                            // exists in scene
                            unsigned int geom_id = *geom_id_opt;
                            // std::cout << "Found joint visual to transform" << std::endl;
                            // std::cout << "- key: " << key << std::endl;
                            // std::cout << "- id: " << geom_id << std::endl;


                            ignition::math::Pose3d link_world_pose = link->WorldPose();
                            msgs::Pose vis_pose = vis.pose();

                            // convert to rmagine
                            rm::Transform Tlw = to_rm(link_world_pose);
                            rm::Transform Tvl = to_rm(vis_pose);

                            rm::Transform Tvw = Tlw * Tvl;
                            // std::cout << "- transform: " << Tvw << std::endl;

                            auto Tiv = m_geom_to_visual[geom].T;
                            geom->setTransform(Tvw * Tiv);
                            mesh_links_to_update.insert(geom);

                        } else {
                            std::cout << "[RmagineEmbreeMap] WARNING: visual not in mesh set. But it should." << std::endl;
                            std::cout << "[RmagineEmbreeMap] - key: " << key << std::endl;
                        }
                    }
                }

                // TODO: link->GetChildJointsLinks() for all other visuals to update
            } else {
                std::cout << "[RmagineEmbreeMap] WARNING: Could not find link " << link_name << " of model " << model->GetName() << std::endl; 
            }
        }
    }

    return mesh_links_to_update;
}

void RmagineEmbreeMap::UpdateState()
{
    
    // std::cout << "UpdateState" << std::endl;
    std::vector<physics::ModelPtr> models = m_world->Models();
    std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);
    updateModelIgnores(models_new, m_model_ignores);

    SceneDiff diff = m_scene_state.diff(models_new, 
            m_changed_delta_trans, 
            m_changed_delta_rot, 
            m_changed_delta_scale);

    // apply changes to rmagine
    if(diff.HasChanged())
    {
        
        // TODO! translate gazebo models to embree map instances
        // std::cout << "[RmagineEmbreeMap] SCENE HAS CHANGED" << std::endl;
        // std::cout << diff << std::endl;

        size_t scene_changes = 0;

        std::unordered_map<rm::EmbreeGeometryPtr, VisualTransform> updates_add;

        // Insert new meshes
        if(diff.ModelAdded())
        {
            std::cout << "[RmagineEmbreeMap] 1. ADD MODELS" << std::endl;
            updates_add = EmbreeUpdateAdded(models_new, diff.added);
            scene_changes += updates_add.size();

            for(auto elem : updates_add)
            {
                rm::EmbreeGeometryPtr geom = elem.first;
                std::string key = elem.second.name;
                rm::Transform T = elem.second.T;
                uint32_t model_id = elem.second.model_id;

                // create global double connection between visual and embree geometries
                if(m_visual_to_geoms.find(key) == m_visual_to_geoms.end())
                {
                    m_visual_to_geoms[key] = {};
                }
                m_visual_to_geoms[key].push_back(geom);

                if(m_model_meshes.find(model_id) == m_model_meshes.end())
                {
                    m_model_meshes[model_id] = {};
                }
                m_model_meshes[model_id].push_back(geom);

                // insert global
                m_geom_to_visual[geom] = elem.second;
            }

            // TODO: check 
            // error scene not committed sometimes when adding meshes. Could result in segfaults. 
            // std::cout << "1. models added." << std::endl;
        }

        
        if(updates_add.size() > 0)
        {
            // apply updates add to map (locked)
            if(m_map_mutex)
            {
                m_map_mutex->lock();
            }
            for(auto elem : updates_add)
            {
                unsigned int geom_id = m_map->scene->add(elem.first);
            }
            if(m_map_mutex)
            {
                m_map_mutex->unlock();
            }

            gzdbg << "[RmagineEmbreeMap] New map elements" << std::endl;
            gzdbg << "[RmagineEmbreeMap] - geometries: " << m_map->meshes.size() << std::endl;
            gzdbg << "[RmagineEmbreeMap] - instances: " << m_map->scene->geometries().size() << std::endl;
        }

        if(diff.ModelChanged())
        {
            // std::cout << "SCENE CHANGED!" << std::endl;

            // std::cout << "2. UPDATE SCENE - prepare" << std::endl;
            std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_transform;
            // Transform existing meshes
            if(diff.ModelTransformed())
            {
                // std::cout << "2.1. APPLY TRANSFORMATIONS" << std::endl;
                rm::StopWatch sw;
                double el;
                sw();
                meshes_to_transform = EmbreeUpdateTransformed(models_new, diff.transformed);
                el = sw();
                // std::cout << "- Prepare meshes transforms: " << el << " ms" << std::endl;
            }

            std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_scale;
            if(diff.ModelScaled())
            {
                
                rm::StopWatch sw;
                double el;
                sw();
                meshes_to_scale = EmbreeUpdateScaled(models_new, diff.scaled);
                el = sw();

                // if(!meshes_to_scale.empty())
                // {
                //     gzdbg << "[RmagineEmbreeMap] 2.2. APPLY SCALINGS" << std::endl;
                //     gzdbg << "[RmagineEmbreeMap] - Prepare meshes scalings " << meshes_to_scale.size() << ": " << el << " ms" << std::endl;
                // }
            }

            std::unordered_set<rm::EmbreeGeometryPtr> mesh_links_to_update;
            if(diff.ModelJointsChanged())
            {
                rm::StopWatch sw;
                double el;
                sw();
                mesh_links_to_update = EmbreeUpdateJointChanges(models_new, diff.joints_changed);
                el = sw();

                // if(!mesh_links_to_update.empty())
                // {
                //     gzdbg << "[RmagineEmbreeMap] 2.3. APPLY JOINT UPDATES" << std::endl;
                //     gzdbg << "[RmagineEmbreeMap] - Prepare meshes joint updates " << mesh_links_to_update.size() << ": " << el << " ms" << std::endl;
                // }
            }

            // mutex?
            rm::StopWatch sw;
            double el;

            auto meshes_to_update = get_union(meshes_to_transform, meshes_to_scale);
            meshes_to_update = get_union(meshes_to_update, mesh_links_to_update);

            // sw();
            for(auto mesh_to_update : meshes_to_update)
            {
                mesh_to_update->apply();
                mesh_to_update->commit();
                scene_changes++;
            }

            // el = sw();
            // std::cout << "- Geometry updates: " << el << "s" << std::endl;
        }
        
        if(diff.ModelRemoved())
        {
            // std::cout << "3. APPLY REMOVALS" << std::endl;
            for(auto geom_id : diff.removed)
            {
                if(m_model_ignores.find(geom_id) != m_model_ignores.end())
                {
                    continue;
                }

                auto geoms = m_model_meshes[geom_id];
                // std::cout << "Remove " << geoms.size() << " geometries" << std::endl;

                for(auto geom : geoms)
                {
                    // TODO why does the remove needs the mutex?
                    // tought that the scene->commit would commit all the changes
                    if(m_map_mutex)
                    {
                        m_map_mutex->lock();
                    }

                    m_map->scene->remove(geom);

                    if(m_map_mutex)
                    {
                        m_map_mutex->unlock();
                    }

                    scene_changes++;
                }
            }
        }

        if(scene_changes > 0)
        {
            rm::StopWatch sw;
            double el;
            
            if(m_map_mutex)
            {
                m_map_mutex->lock();
            }

            // std::cout << "SCENE UPDATE: " << scene_changes << " changes" << std::endl;

            sw();
            m_map->scene->commit();
            el = sw();
            // std::cout << "- Scene update finished in " << el << "s" << std::endl;
            
            if(m_map_mutex)
            {
                m_map_mutex->unlock();
            }

            // std::cout << "Scene Info: " << std::endl;
            // std::cout << "- instances: " << m_map->scene->count<rm::EmbreeInstance>() << std::endl;
            // std::cout << "- meshes: " << m_map->scene->count<rm::EmbreeMesh>() << std::endl;
        }
    }

    m_sensors_loaded = !m_scene_state.update(models_new, diff);

    if(!m_sensors_loaded)
    {
        // std::cout << "Reload sensors!" << std::endl;
    }
}

void RmagineEmbreeMap::UpdateSensors()
{
    // TODO: get all sensors of certain type, simulate sensor data if required   
    if(!m_sensors_loaded)
    {
        std::vector<sensors::SensorPtr> sensors 
            = sensors::SensorManager::Instance()->GetSensors();

        for(sensors::SensorPtr sensor : sensors)
        {
            sensors::RmagineEmbreeSphericalPtr spherical 
                = std::dynamic_pointer_cast<sensors::RmagineEmbreeSpherical>(sensor);

            if(spherical)
            {
                std::cout << "[RmagineEmbreeMap] Found Rmagine spherical sensor " << spherical->ScopedName() << std::endl;
                spherical->setMap(m_map);
                if(!m_map_mutex)
                {
                    std::cout << "[RmagineEmbreeMap] no mutex " << std::endl;
                }
                spherical->setLock(m_map_mutex);
            }
        }
        m_sensors_loaded = true;
    }
}

void RmagineEmbreeMap::OnWorldUpdate(const common::UpdateInfo& info)
{
    if(!m_updater_thread.valid() 
    || m_updater_thread.wait_for(0ms) == std::future_status::ready)
    {
        // TODO: dont compute this twice!
        std::vector<physics::ModelPtr> models = m_world->Models();
        std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);
        updateModelIgnores(models_new, m_model_ignores);

        SceneDiff diff = m_scene_state.diff(models_new, 
            m_changed_delta_trans, 
            m_changed_delta_rot, 
            m_changed_delta_scale);

        if(diff.HasChanged())
        {
            m_updater_thread = std::async(std::launch::async, [this] {
                    UpdateState();
                    UpdateSensors();
                });
        }
    }
}

GZ_REGISTER_WORLD_PLUGIN(RmagineEmbreeMap)

} // namespace gazebo