#include <rmagine_gazebo_plugins/rmagine_optix_map_gzplugin.h>

#include <rmagine_gazebo_plugins/helper/conversions.h>
#include <rmagine_gazebo_plugins/helper/optix_conversions.h>

#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/common/Console.hh>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>

#include <iostream>
#include <chrono>
#include <functional>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/optix/OptixInst.hpp>

#include <rmagine/map/optix/optix_shapes.h>


using namespace std::placeholders;
using namespace boost::algorithm;
using namespace std::chrono_literals;

namespace rm = rmagine;

namespace gazebo
{

RmagineOptixMap::RmagineOptixMap()
{
    gzdbg << "[RmagineOptixMap] Constructed." << std::endl;
}

RmagineOptixMap::~RmagineOptixMap()
{
    if(m_updater_thread.joinable())
    {
        m_stop_updater_thread = true;
        m_updater_thread.join();
    }
    gzdbg << "[RmagineOptixMap] Destroyed." << std::endl;
}

void RmagineOptixMap::Load(
    physics::WorldPtr _world, 
    sdf::ElementPtr _sdf)
{
    m_map_mutex = std::make_shared<std::shared_mutex>();
    m_world = _world;
    m_sdf = _sdf;

    parseParams(_sdf);

    // create empty map
    rm::OptixScenePtr scene = std::make_shared<rm::OptixScene>();

    // rm::OptixInstancesPtr insts = std::make_shared<rm::OptixInstances>();
    // scene->setRoot(insts);

    m_map = std::make_shared<rm::OptixMap>(scene);


    // gzdbg << "Starting updater thread." << std::endl;

    m_updater_thread = std::thread([this]()
    {
        // gzdbg << "Updater thread started." << std::endl;
        rm::StopWatch sw;
        double el;
        // minimum duration for one loop
        double el_min = 1.0 / m_update_rate_limit;
        
        while(!m_stop_updater_thread)
        {
            sw();
            UpdateState();
            UpdateSensors();
            el = sw();

            double el_left = el_min - el;
            if(el_left > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(el_left));
            }
        }
        m_stop_updater_thread = false;
        // gzdbg << "Updater thread terminated." << std::endl;
    });

    gzdbg << "[RmagineOptixMap] Loaded." << std::endl;
}

void RmagineOptixMap::parseParams(sdf::ElementPtr sdf)
{
    if(sdf->HasElement("update"))
    {
        sdf::ElementPtr updateElem = sdf->GetElement("update");
    
        if(updateElem->HasElement("rate_limit"))
        {
            m_update_rate_limit = updateElem->Get<double>("rate_limit");
        }

        if(updateElem->HasElement("delta_trans"))
        {
            m_changed_delta_trans = updateElem->Get<double>("delta_trans");
        }

        if(updateElem->HasElement("delta_rot"))
        {
            m_changed_delta_rot = updateElem->Get<double>("delta_rot");
        }

        if(updateElem->HasElement("delta_scale"))
        {
            m_changed_delta_scale = updateElem->Get<double>("delta_scale");
        }
    }
}

std::unordered_map<rm::OptixInstPtr, VisualTransform> RmagineOptixMap::OptixUpdateAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& added)
{
    std::unordered_map<rm::OptixInstPtr, VisualTransform> inst_to_visual;

    for(auto model_id : added)
    {
        auto model_it = models.find(model_id);
        if(model_it == models.end())
        {
            continue;
        }
        physics::ModelPtr model = model_it->second;
        if(!model)
        {
            gzwarn << "WARNING - OptixUpdateAdded: model empty " << std::endl;
            continue;
        }

        sdf::ElementPtr modelElem = model->GetSDF();
        if(modelElem && modelElem->HasElement("rmagine_ignore"))
        {
            m_model_ignores.insert(model_id);
            continue;
        }


        std::string model_name = model->GetName();
        // std::cout << "ADDING " << model_name << std::endl;

        std::vector<physics::LinkPtr> links = model->GetLinks();
        for(physics::LinkPtr link : links)
        {
            if(!link)
            {
                gzwarn << "WARNING - OptixUpdateAdded: link empty" << std::endl;
                continue;
            }

            std::string link_name = link->GetName();

            sdf::ElementPtr linkElem = link->GetSDF();
            if(linkElem->HasElement("rmagine_ignore"))
            {
                m_link_ignores.insert(link->GetScopedName());
                m_model_has_link_ignores.insert(model_id);
                continue;
            }

            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();
            ignition::math::Pose3d link_world_pose = link->WorldPose();
            rm::Transform Tlw = to_rm(link_world_pose);

            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                msgs::Vector3d vis_scale = vis.scale();
                msgs::Pose vis_pose = vis.pose();
                std::string vis_name = vis.name();

                rm::Vector Svl = to_rm(vis_scale);

                if(Svl.x > 1.001 || Svl.y > 1.001 || Svl.z > 1.001
                    || Svl.x < 0.999 || Svl.y < 0.999 || Svl.z < 0.999)
                {
                    gzwarn << "[RmagineOptixMap] WARNING: Scale from visual to link currently unused but it seems to be set to " << Svl << std::endl; 
                }

                rm::Transform Tvl = to_rm(vis_pose);
                rm::Transform Tvw = Tlw * Tvl;

                if(vis.has_geometry())
                {
                    msgs::Geometry gzgeom = vis.geometry();

                    std::vector<rm::OptixInstPtr> insts;
                    std::unordered_set<rm::OptixInstPtr> insts_ignore_model_transform;

                    if(gzgeom.has_box())
                    {
                        msgs::BoxGeom gzbox = gzgeom.box();

                        
                        rm::OptixScenePtr box_scene;
                        

                        auto cache_it = m_geom_cache.find(GeomCacheID::BOX);
                        if(cache_it != m_geom_cache.end())
                        {
                            box_scene = cache_it->second;
                        } else {
                            rm::OptixGeometryPtr box_geom = std::make_shared<rm::OptixCube>();
                            box_geom->apply();
                            box_geom->commit();
                            box_scene = box_geom->makeScene();
                            box_scene->commit();
                            m_geom_cache[GeomCacheID::BOX] = box_scene;
                        }

                        rm::OptixInstPtr mesh_inst = box_scene->instantiate();
                        msgs::Vector3d size = gzbox.size();
                        rm::Vector3 rm_scale = to_rm(size);
                        mesh_inst->setScale(rm_scale);
                        
                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_cylinder())
                    {
                        msgs::CylinderGeom gzcylinder = gzgeom.cylinder();

                        auto cache_it = m_geom_cache.find(GeomCacheID::CYLINDER);

                        rm::OptixScenePtr cylinder_scene;
                        
                        if(cache_it != m_geom_cache.end())
                        {
                            cylinder_scene = cache_it->second;
                        } else {
                            rm::OptixGeometryPtr cylinder_geom = std::make_shared<rm::OptixCylinder>(100);
                            cylinder_geom->apply();
                            cylinder_geom->commit();
                            cylinder_scene = cylinder_geom->makeScene();
                            cylinder_scene->commit();
                            m_geom_cache[GeomCacheID::CYLINDER] = cylinder_scene;
                        }

                        rm::OptixInstPtr mesh_inst = cylinder_scene->instantiate();
                        float radius = gzcylinder.radius();
                        float diameter = radius * 2.0;
                        float height = gzcylinder.length();
                        mesh_inst->setScale({diameter, diameter, height});

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_sphere())
                    {
                        msgs::SphereGeom gzsphere = gzgeom.sphere();
                        
                        auto cache_it = m_geom_cache.find(GeomCacheID::SPHERE);

                        rm::OptixScenePtr sphere_scene;
                        if(cache_it != m_geom_cache.end())
                        {
                            sphere_scene = cache_it->second;
                        } else {
                            rm::OptixGeometryPtr sphere_geom = std::make_shared<rm::OptixSphere>(30, 30);
                            sphere_geom->apply();
                            sphere_geom->commit();
                            sphere_scene = sphere_geom->makeScene();
                            sphere_scene->commit();
                            m_geom_cache[GeomCacheID::SPHERE] = sphere_scene;
                        }

                        rm::OptixInstPtr mesh_inst = sphere_scene->instantiate();
                        float diameter = gzsphere.radius() * 2.0;
                        mesh_inst->setScale({diameter, diameter, diameter});

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_plane())
                    {
                        msgs::PlaneGeom gzplane = gzgeom.plane();
                        
                        auto cache_it = m_geom_cache.find(GeomCacheID::PLANE);
                        rm::OptixScenePtr plane_scene;
                        if(cache_it != m_geom_cache.end())
                        {
                            plane_scene = cache_it->second;
                        } else {
                            
                            // std::cout << "Make plane geom." << std::endl;
                            rm::OptixGeometryPtr plane_geom = std::make_shared<rm::OptixPlane>();
                            plane_geom->apply();
                            plane_geom->commit();
                            // std::cout << "Make plane geom done." << std::endl;
                            // std::cout << "Make Plane Scene" << std::endl;
                            plane_scene = plane_geom->makeScene();
                            plane_scene->commit();
                            m_geom_cache[GeomCacheID::PLANE] = plane_scene;
                            // std::cout << "Make plane scene done." << std::endl;
                        }

                        msgs::Vector2d size = gzplane.size();

                        rm::OptixInstPtr mesh_inst = plane_scene->instantiate();

                        rm::Vector3 scale;
                        scale.x = size.x();
                        scale.y = size.y();
                        scale.z = 1.0;
                        mesh_inst->setScale(scale);

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_heightmap())
                    {
                        // model pose is ignored for heightmap!
                        msgs::HeightmapGeom gzheightmap = gzgeom.heightmap();

                        rm::OptixGeometryPtr geom = to_rm_optix(gzheightmap);

                        if(geom)
                        {
                            geom->commit();
                            rm::OptixInstPtr geom_inst = geom->instantiate();
                            
                            insts.push_back(geom_inst);
                            insts_ignore_model_transform.insert(geom_inst);
                        } else {
                            gzwarn << "[RmagineOptixMap] Could not load heightmap!" << std::endl;  
                        }
                    }

                    if(gzgeom.has_mesh())
                    {
                        msgs::MeshGeom gzmesh = gzgeom.mesh();
                        rm::Vector3 scale = to_rm(gzmesh.scale());

                        rm::OptixScenePtr scene;

                        auto cache_it = m_mesh_cache.find(gzmesh.filename());
                        if(cache_it != m_mesh_cache.end())
                        {
                            scene = cache_it->second;
                        } else {
                            for(auto loader_it = m_mesh_loader.begin(); loader_it != m_mesh_loader.end() && !scene; ++loader_it)
                            {
                                if(*loader_it == MeshLoading::INTERNAL)
                                {
                                    scene = to_rm_optix_assimp(gzmesh);
                                } else if(*loader_it == MeshLoading::GAZEBO) {
                                    scene = to_rm_optix_gazebo(gzmesh);
                                }
                            }

                            if(scene)
                            {
                                scene->commit();
                                m_mesh_cache[gzmesh.filename()] = scene;
                            }
                        }

                        if(scene)
                        {
                            rm::OptixInstPtr inst = scene->instantiate();
                            inst->setScale(scale);
                            insts.push_back(inst);
                        } else {
                            gzwarn << "[RmagineOptixMap] WARNING add mesh failed. Could not load " << gzmesh.filename() << std::endl;
                        }
                    }

                    for(auto inst : insts)
                    {
                        auto Tiv = inst->transform();
                        inst->name = vis_name;
                        
                        if(insts_ignore_model_transform.find(inst) == insts_ignore_model_transform.end())
                        {    
                            auto Tiw = Tvw * Tiv;
                            inst->setTransform(Tiw);
                        }
                        
                        inst->apply();
                        inst->commit();
                        inst_to_visual[inst] = {vis_name, Tiv, model_id};
                    }
                }
            }
        }


        // std::cout << "ADDING " << model_name << " done." << std::endl;
    }

    return inst_to_visual;
}

std::unordered_set<rm::OptixInstPtr> RmagineOptixMap::OptixUpdateTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& transformed)
{
    std::unordered_set<rm::OptixInstPtr> insts_to_transform;

    // rm::OptixInstancesPtr insts_global = 
    //     std::dynamic_pointer_cast<rm::OptixInstances>(
    //         m_map->scene()->getRoot());

    for(auto model_id : transformed)
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

        ignition::math::Vector3d model_scale_gz = model->Scale();
        rm::Vector3 model_scale = to_rm(model_scale_gz);

        for(physics::LinkPtr link : links)
        {
            if(!link)
            {
                gzwarn << "WARNING - OptixUpdateTransformed: link empty" << std::endl;
                continue;
            }

            if(m_link_ignores.find(link->GetScopedName()) != m_link_ignores.end())
            {
                continue;
            }

            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                std::string key = vis.name();

                auto mesh_vis_it = m_visual_to_geoms.find(key);
                if(mesh_vis_it == m_visual_to_geoms.end())
                {
                    gzwarn << "[RmagineOptixMap] WARNING mesh to update not found in embree. Skipping." << std::endl;
                    gzwarn << "- key: " << key << std::endl;
                    continue;
                }

                for(auto inst : mesh_vis_it->second)
                {
                    auto inst_id_opt = m_map->scene()->getOpt(inst);

                    if(inst_id_opt)
                    {
                        unsigned int inst_id = *inst_id_opt;

                        ignition::math::Pose3d link_world_pose = link->WorldPose();
                        msgs::Pose vis_pose = vis.pose();

                        rm::Transform Tlw = to_rm(link_world_pose);
                        rm::Transform Tvl = to_rm(vis_pose);

                        rm::Transform Tvw = Tlw * Tvl;

                        auto Tiv = m_geom_to_visual[inst].T;
                        inst->setTransform(Tvw * Tiv);
                        insts_to_transform.insert(inst);

                    } else {
                        gzwarn << "WARNING - OptixUpdateTransformed: visual not in scene. But it should." << std::endl;
                        gzwarn << "- key: " << key << std::endl;
                    }

                }
            }
        }
    }

    return insts_to_transform;
}

std::unordered_set<rmagine::OptixInstPtr> RmagineOptixMap::OptixUpdateScaled(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& scaled)
{
    std::unordered_set<rm::OptixInstPtr> insts_to_scale;

    for(auto model_id : scaled)
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

        ignition::math::Vector3d model_scale_gz = model->Scale();
        rm::Vector3 model_scale = to_rm(model_scale_gz);

        for(physics::LinkPtr link : links)
        {
            if(!link)
            {
                gzwarn << "WARNING - OptixUpdateScaled: link empty" << std::endl;
                continue;
            }

            if(m_link_ignores.find(link->GetScopedName()) != m_link_ignores.end())
            {
                continue;
            }

            std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

            for(auto elem : visuals)
            {
                msgs::Visual vis = elem.second;
                std::string key = vis.name();

                auto mesh_vis_it = m_visual_to_geoms.find(key);
                if(mesh_vis_it == m_visual_to_geoms.end())
                {
                    gzwarn << "[RmagineOptixMap] WARNING mesh to update not found in embree. Skipping." << std::endl;
                    gzwarn << "[RmagineOptixMap] - key: " << key << std::endl;
                    continue;
                }

                for(auto inst : mesh_vis_it->second)
                {
                    auto inst_id_opt = m_map->scene()->getOpt(inst);

                    if(inst_id_opt)
                    {
                        unsigned int inst_id = *inst_id_opt;

                        msgs::Vector3d vis_scale = vis.scale();

                        inst->setScale(model_scale);
                        insts_to_scale.insert(inst);
                    } else {
                        gzwarn << "WARNING - OptixUpdateScaled: visual not in scene. But it should." << std::endl;
                        gzwarn << "- key: " << key << std::endl;
                    }
                }
            }
        }
    }

    return insts_to_scale;
}

std::unordered_set<rm::OptixInstPtr> RmagineOptixMap::OptixUpdateJointChanges(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_map<uint32_t, std::unordered_set<std::string> >& joints_changed)
{
    std::unordered_set<rm::OptixInstPtr> inst_links_to_update;

    for(auto elem : joints_changed)
    {
        uint32_t model_id = elem.first;
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

        if(!model)
        {
            gzwarn << "WARNING - OptixUpdateJointChanges: Model empty" << std::endl;
            continue;
        }

        for(std::string link_name : elem.second)
        {
            physics::LinkPtr link = model->GetLink(link_name);
            if(link)
            {
                if(m_link_ignores.find(link->GetScopedName()) != m_link_ignores.end())
                {
                    continue;
                }

                std::map<uint32_t, msgs::Visual> visuals = link->Visuals();
                for(auto elem : visuals)
                {
                    msgs::Visual vis = elem.second;
                    std::string key = vis.name();

                    auto mesh_vis_it = m_visual_to_geoms.find(key);

                    if(mesh_vis_it == m_visual_to_geoms.end())
                    {
                        gzwarn << "[RmagineOptixMap] WARNING mesh to update not found in optix. Skipping." << std::endl;
                        gzwarn << "[RmagineOptixMap] - key: " << key << std::endl;
                        continue;
                    }

                    for(auto inst : mesh_vis_it->second)
                    {
                        auto geom_id_opt = m_map->scene()->getOpt(inst);

                        if(geom_id_opt)
                        {
                            unsigned int geom_id = *geom_id_opt;

                            ignition::math::Pose3d link_world_pose = link->WorldPose();
                            msgs::Pose vis_pose = vis.pose();

                            // convert to rmagine
                            rm::Transform Tlw = to_rm(link_world_pose);
                            rm::Transform Tvl = to_rm(vis_pose);
                            rm::Transform Tvw = Tlw * Tvl;

                            auto Tiv = m_geom_to_visual[inst].T;
                            inst->setTransform(Tvw * Tiv);
                            inst_links_to_update.insert(inst);

                        } else {
                            gzwarn << "WARNING - OptixUpdateJointChanges: visual not in scene. But it should." << std::endl;
                            gzwarn << "- key: " << key << std::endl;
                        }
                    }
                }
            } else {
                gzwarn << "[RmagineOptixMap] WARNING: Could not find link " << link_name << " of model " << model->GetName() << std::endl; 
            }
        }
    }

    return inst_links_to_update;

}

void RmagineOptixMap::UpdateState()
{
    std::vector<physics::ModelPtr> models = m_world->Models();
    std::unordered_map<uint32_t, physics::ModelPtr> models_new;

    { // ToIdMap
        for(size_t i=0; i<models.size() && i<models.capacity(); i++)
        {
            physics::ModelPtr model = models[i];
            if(model)
            {
                uint32_t model_id = model->GetId();
                models_new[model_id] = model;
            }
        }
    }

    // updateModelIgnores(models_new, m_model_ignores);

    rm::StopWatch sw;
    double el;
    sw();

    SceneDiff diff = m_scene_state.diff(models_new, 
            m_changed_delta_trans, 
            m_changed_delta_rot, 
            m_changed_delta_scale);
    
    el = sw();
    

    // apply changes to rmagine
    if(diff.HasChanged())
    {
        // std::cout << "UPDATE OPTIX SCENE" << std::endl;
        // std::cout << "Computed diff in " << el << "s" << std::endl;
        // 
        // std::cout << diff << std::endl;

        // count total scene changes
        size_t scene_changes = 0;

        std::unordered_map<rm::OptixInstPtr, VisualTransform> updates_add;

        if(diff.ModelAdded())
        {
            // std::cout << "OptixUpdateAdded" << std::endl;
            updates_add = OptixUpdateAdded(models_new, diff.added);
            // std::cout << "OptixUpdateAdded done." << std::endl;
            scene_changes += updates_add.size();

            for(auto elem : updates_add)
            {
                rm::OptixInstPtr inst = elem.first;
                std::string key = elem.second.name;
                rm::Transform T = elem.second.T;
                uint32_t model_id = elem.second.model_id;

                // TODO
                if(m_visual_to_geoms.find(key) == m_visual_to_geoms.end())
                {
                    m_visual_to_geoms[key] = {};
                }
                m_visual_to_geoms[key].push_back(inst);

                if(m_model_meshes.find(model_id) == m_model_meshes.end())
                {
                    m_model_meshes[model_id] = {};
                }
                m_model_meshes[model_id].push_back(inst);
                // insert global
                m_geom_to_visual[inst] = elem.second;
            }
        }
        
        if(updates_add.size() > 0)
        {
            // std::cout << "ADD " << updates_add.size() << " instances." << std::endl;
            for(auto elem : updates_add)
            {
                rm::OptixInstPtr inst = elem.first;
                unsigned int inst_id = m_map->scene()->add(inst);
            }
            // std::cout << "ADD " << updates_add.size() << " instances done." << std::endl;
        }

        if(diff.ModelChanged())
        {
            std::unordered_set<rm::OptixInstPtr> insts_to_transform;

            rm::StopWatch sw;
            double el;

            sw();
            if(diff.ModelTransformed())
            {
                insts_to_transform = OptixUpdateTransformed(models_new, diff.transformed);
            }

            std::unordered_set<rm::OptixInstPtr> insts_to_scale;
            if(diff.ModelScaled())
            {
                insts_to_scale = OptixUpdateScaled(models_new, diff.scaled);
            }
            
            std::unordered_set<rm::OptixInstPtr> inst_links_to_update;
            if(diff.ModelJointsChanged())
            {
                inst_links_to_update = OptixUpdateJointChanges(models_new, diff.joints_changed);
            }

            auto meshes_to_update = get_union(insts_to_transform, insts_to_scale);
            meshes_to_update = get_union(meshes_to_update, inst_links_to_update);

            for(auto mesh_to_update : meshes_to_update)
            {
                mesh_to_update->apply();
                scene_changes++;
            }

            el = sw();
        }

        if(diff.ModelRemoved())
        {
            // TODO: remove meshes if no one uses them anymore

            for(auto model_id : diff.removed)
            {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
                    continue;
                }

                auto insts_it = m_model_meshes.find(model_id);

                if(insts_it != m_model_meshes.end())
                {
                    auto insts = insts_it->second;
                    for(auto inst : insts)
                    {
                        m_map->scene()->remove(inst);
                        scene_changes++;

                        // REMOVE VISUAL CONNECTIONS
                        {
                            auto vis_it = m_geom_to_visual.find(inst);
                            if(vis_it != m_geom_to_visual.end())
                            {
                                std::string key = vis_it->second.name;

                                // std::cout << "ERASE instance from instance->visual map. visual: " << key << std::endl;

                                m_geom_to_visual.erase(vis_it);

                                auto geom_it = m_visual_to_geoms.find(key);

                                if(geom_it != m_visual_to_geoms.end())
                                {
                                    // ERASE ALL GEOMETRIES AT ONCE
                                    m_visual_to_geoms.erase(geom_it);
                                }
                            } else {
                                gzwarn << "WARNING: instance to remove was not in m_geom_to_visual" << std::endl;
                            }
                        }
                    }

                    // this causes a segfault
                    // m_model_meshes.erase(insts_it);
                } else {
                    gzwarn << "WARNING: Could not found model (" << model_id << ") in m_model_meshes." << std::endl;
                }

                // REMOVE LINK IGNORES
                {
                    auto model_has_link_ignores_it = m_model_has_link_ignores.find(model_id);
                    if(model_has_link_ignores_it != m_model_has_link_ignores.end())
                    {
                        // HAS LINK IGNORES
                        auto model_it = models_new.find(model_id);
                        if(model_it != models_new.end())
                        {
                            physics::ModelPtr model = model_it->second;
                            std::vector<physics::LinkPtr> links = model->GetLinks();
                            for(physics::LinkPtr link : links)
                            {
                                if(!link)
                                {
                                    gzwarn << "WARNING - EmbreeUpdateAdded: link empty " << std::endl;
                                    continue;
                                }

                                m_link_ignores.erase(link->GetScopedName());
                            }
                        }

                        m_model_has_link_ignores.erase(model_has_link_ignores_it);
                    }
                }

            }
        }

        if(diff.ModelAdded() || diff.ModelRemoved())
        {
            gzdbg << "[RmagineOptixMap] Number of map elements changed" << std::endl;
            gzdbg << "[RmagineOptixMap] - instances: " << m_map->scene()->geometries().size() << std::endl;
            // gzdbg << "[RmagineOptixMap] - instances: " << insts_old->instances().size() << std::endl;
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
            m_map->scene()->commit();
            el = sw();
            // std::cout << "- Scene update finished in " << el << "s" << std::endl;

            if(m_map_mutex)
            {
                m_map_mutex->unlock();
            }
        }
    }

    m_sensors_loaded = !m_scene_state.update(models_new, diff);

    if(!m_sensors_loaded)
    {
        // std::cout << "Reload sensors!" << std::endl;
    }
}

void RmagineOptixMap::UpdateSensors()
{
    if(!m_sensors_loaded)
    {
        std::vector<sensors::SensorPtr> sensors 
            = sensors::SensorManager::Instance()->GetSensors();

        for(sensors::SensorPtr sensor : sensors)
        {
            sensors::RmagineOptixSphericalPtr spherical 
                = std::dynamic_pointer_cast<sensors::RmagineOptixSpherical>(sensor);

            if(spherical)
            {
                gzdbg << "[RmagineOptixMap] Found Rmagine spherical sensor " << spherical->ScopedName() << std::endl;
                spherical->setLock(m_map_mutex);
                spherical->setMap(m_map);
                if(!m_map_mutex)
                {
                    gzwarn << "[RmagineOptixMap] no mutex " << std::endl;
                }
                
            }
        }
        m_sensors_loaded = true;
    }
}

GZ_REGISTER_WORLD_PLUGIN(RmagineOptixMap)

} // namespace gazebo