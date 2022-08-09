#include <rmagine_gazebo_plugins/rmagine_optix_map_gzplugin.h>

#include <rmagine_gazebo_plugins/helper/conversions.h>
#include <rmagine_gazebo_plugins/helper/optix_conversions.h>

#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>

#include <iostream>
#include <chrono>
#include <functional>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/optix/OptixInstances.hpp>
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
    std::cout << "[RmagineOptixMap] Constructed." << std::endl;
}

RmagineOptixMap::~RmagineOptixMap()
{
    std::cout << "[RmagineOptixMap] Destroyed." << std::endl;
}

void RmagineOptixMap::Load(
    physics::WorldPtr _world, 
    sdf::ElementPtr _sdf)
{
    m_map_mutex = std::make_shared<std::shared_mutex>();
    m_world = _world;
    m_sdf = _sdf;

    m_world_update_conn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RmagineOptixMap::OnWorldUpdate, this, std::placeholders::_1)
    );

    // create empty map
    rm::OptixScenePtr scene = std::make_shared<rm::OptixScene>();

    rm::OptixInstancesPtr insts = std::make_shared<rm::OptixInstances>();
    scene->setRoot(insts);

    m_map = std::make_shared<rm::OptixMap>(scene);
    std::cout << "[RmagineOptixMap] Loaded." << std::endl;
}

std::unordered_map<rm::OptixInstPtr, VisualTransform> RmagineOptixMap::OptixUpdateAdded(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& added)
{
    std::unordered_map<rm::OptixInstPtr, VisualTransform> inst_to_visual;


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
        std::cout << "Add model: " << model_name << std::endl;

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
                    std::cout << "WARNING: Scale from visual to link currently unused but it seems to be set to " << Svl << std::endl; 
                }

                rm::Transform Tvl = to_rm(vis_pose);
                rm::Transform Tvw = Tlw * Tvl;
                std::string key = vis.name();

                if(vis.has_geometry())
                {
                    msgs::Geometry gzgeom = vis.geometry();

                    std::vector<rm::OptixInstPtr> insts;
                    std::unordered_set<rm::OptixInstPtr> insts_ignore_model_transform;

                    if(gzgeom.has_box())
                    {
                        std::cout << "ADD BOX!" << std::endl;

                        msgs::BoxGeom gzbox = gzgeom.box();

                        auto cache_it = m_geom_cache.find(GeomCacheID::BOX);

                        rm::OptixGeometryPtr box_geom;

                        if(cache_it != m_geom_cache.end())
                        {
                            box_geom = cache_it->second;
                        } else {
                            box_geom = std::make_shared<rm::OptixCube>();
                            box_geom->apply();
                            box_geom->commit();
                            m_geom_cache[GeomCacheID::BOX] = box_geom;
                        }

                        rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
                        mesh_inst->setGeometry(box_geom);
                        msgs::Vector3d size = gzbox.size();
                        rm::Vector3 rm_scale = to_rm(size);
                        mesh_inst->setScale(rm_scale);
                        
                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_cylinder())
                    {
                        std::cout << "ADD CYLINDER!" << std::endl;
                        msgs::CylinderGeom gzcylinder = gzgeom.cylinder();

                        auto cache_it = m_geom_cache.find(GeomCacheID::CYLINDER);

                        rm::OptixGeometryPtr cylinder_geom;
                        if(cache_it != m_geom_cache.end())
                        {
                            cylinder_geom = cache_it->second;
                        } else {
                            cylinder_geom = std::make_shared<rm::OptixCylinder>(100);
                            cylinder_geom->apply();
                            cylinder_geom->commit();
                            m_geom_cache[GeomCacheID::CYLINDER] = cylinder_geom;
                        }

                        rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
                        mesh_inst->setGeometry(cylinder_geom);
                        float radius = gzcylinder.radius();
                        float diameter = radius * 2.0;
                        float height = gzcylinder.length();
                        mesh_inst->setScale({diameter, diameter, height});

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_sphere())
                    {
                        std::cout << "ADD SPHERE!" << std::endl;
                        msgs::SphereGeom gzsphere = gzgeom.sphere();
                        
                        auto cache_it = m_geom_cache.find(GeomCacheID::SPHERE);

                        rm::OptixGeometryPtr sphere_geom;
                        if(cache_it != m_geom_cache.end())
                        {
                            sphere_geom = cache_it->second;
                        } else {
                            sphere_geom = std::make_shared<rm::OptixSphere>(30, 30);
                            sphere_geom->apply();
                            sphere_geom->commit();
                            m_geom_cache[GeomCacheID::SPHERE] = sphere_geom;
                        }

                        rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
                        mesh_inst->setGeometry(sphere_geom);
                        float diameter = gzsphere.radius() * 2.0;
                        mesh_inst->setScale({diameter, diameter, diameter});

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_plane())
                    {
                        std::cout << "ADD PLANE!" << std::endl;
                        msgs::PlaneGeom gzplane = gzgeom.plane();
                        
                        auto cache_it = m_geom_cache.find(GeomCacheID::PLANE);
                        rm::OptixGeometryPtr plane_geom;
                        if(cache_it != m_geom_cache.end())
                        {
                            plane_geom = cache_it->second;
                        } else {
                            plane_geom = std::make_shared<rm::OptixPlane>();
                            plane_geom->apply();
                            plane_geom->commit();
                            m_geom_cache[GeomCacheID::PLANE] = plane_geom;
                        }

                        msgs::Vector2d size = gzplane.size();

                        rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
                        mesh_inst->setGeometry(plane_geom);

                        rm::Vector3 scale;
                        scale.x = size.x();
                        scale.y = size.y();
                        scale.z = 1.0;
                        mesh_inst->setScale(scale);

                        insts.push_back(mesh_inst);
                    }

                    if(gzgeom.has_heightmap())
                    {
                        std::cout << "ADD HEIGHTMAP!" << std::endl;

                        // model pose is ignored for heightmap!
                        msgs::HeightmapGeom gzheightmap = gzgeom.heightmap();

                        rm::OptixGeometryPtr geom = to_rm(gzheightmap);

                        // make instance
                        rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
                        mesh_inst->setGeometry(geom);
                        
                        insts.push_back(mesh_inst);
                        insts_ignore_model_transform.insert(mesh_inst);
                    }

                    if(gzgeom.has_mesh())
                    {
                        std::cout << "ADD MESH!" << std::endl;
                        msgs::MeshGeom gzmesh = gzgeom.mesh();
                        rm::Vector3 scale = to_rm(gzmesh.scale());

                        rm::OptixScenePtr scene;

                        auto cache_it = m_mesh_cache.find(gzmesh.filename());
                        if(cache_it != m_mesh_cache.end())
                        {
                            std::cout << "USING CACHED MODEL" << std::endl;
                            scene = cache_it->second;
                        } else {
                            scene = to_rm(gzmesh);
                            m_mesh_cache[gzmesh.filename()] = scene;
                        }

                        rm::OptixGeometryPtr root = scene->getRoot();

                        rm::OptixInstancesPtr insts_new = std::dynamic_pointer_cast<rm::OptixInstances>(root);
                        if(insts_new)
                        {
                            for(auto elem : insts_new->instances())
                            {
                                // make a copy
                                rm::OptixInstPtr inst = std::make_shared<rm::OptixInst>(*elem.second);

                                // apply scale
                                inst->setScale(inst->scale().mult_ewise(scale));
                                insts.push_back(inst);
                            }
                        } else {
                            // never change geometry
                            rm::OptixInstPtr inst = std::make_shared<rm::OptixInst>();
                            inst->setGeometry(root);
                            inst->setScale(scale);
                            insts.push_back(inst);
                        }
                    }

                    for(auto inst : insts)
                    {
                        auto Tiv = inst->transform();
                        
                        if(insts_ignore_model_transform.find(inst) == insts_ignore_model_transform.end())
                        {    
                            auto Tiw = Tvw * Tiv;
                            inst->setTransform(Tiw);
                        }
                        
                        inst->apply();

                        inst_to_visual[inst] = {key, Tiv, model_id};
                    }
                }
            }
        }
    }

    return inst_to_visual;
}

std::unordered_set<rm::OptixInstPtr> RmagineOptixMap::OptixUpdateTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models,
    const std::unordered_set<uint32_t>& transformed)
{
    std::unordered_set<rm::OptixInstPtr> insts_to_transform;

    rm::OptixInstancesPtr insts_global = 
        std::dynamic_pointer_cast<rm::OptixInstances>(
            m_map->scene()->getRoot());

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

                for(auto inst : mesh_vis_it->second)
                {
                    unsigned int inst_id = insts_global->get(inst);

                    ignition::math::Pose3d link_world_pose = link->WorldPose();
                    msgs::Pose vis_pose = vis.pose();

                    rm::Transform Tlw = to_rm(link_world_pose);
                    rm::Transform Tvl = to_rm(vis_pose);

                    rm::Transform Tvw = Tlw * Tvl;

                    auto Tiv = m_geom_to_visual[inst].T;
                    inst->setTransform(Tvw * Tiv);
                    insts_to_transform.insert(inst);
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

    rm::OptixInstancesPtr insts_global = 
        std::dynamic_pointer_cast<rm::OptixInstances>(
            m_map->scene()->getRoot());

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

                for(auto inst : mesh_vis_it->second)
                {
                    unsigned int inst_id = insts_global->get(inst);
                    msgs::Vector3d vis_scale = vis.scale();
                    // rm::Vector Svl = to_rm(vis_scale);

                    inst->setScale(model_scale);
                    insts_to_scale.insert(inst);
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

    rm::OptixInstancesPtr insts_global = 
        std::dynamic_pointer_cast<rm::OptixInstances>(
            m_map->scene()->getRoot());

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
                        std::cout << "WARNING mesh to update not found in optix. Skipping." << std::endl;
                        std::cout << "- key: " << key << std::endl;
                        continue;
                    }

                    for(auto inst : mesh_vis_it->second)
                    {
                        unsigned int geom_id = insts_global->get(inst);
                        
                        ignition::math::Pose3d link_world_pose = link->WorldPose();
                        msgs::Pose vis_pose = vis.pose();

                        // convert to rmagine
                        rm::Transform Tlw = to_rm(link_world_pose);
                        rm::Transform Tvl = to_rm(vis_pose);
                        rm::Transform Tvw = Tlw * Tvl;

                        auto Tiv = m_geom_to_visual[inst].T;
                        inst->setTransform(Tvw * Tiv);
                        inst_links_to_update.insert(inst);
                    }
                }
            } else {
                std::cout << "WARNING: Could not find link " << link_name << " of model " << model->GetName() << std::endl; 
            }
        }
    }

    return inst_links_to_update;

}

void RmagineOptixMap::UpdateState()
{
    std::vector<physics::ModelPtr> models = m_world->Models();
    std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);
    updateModelIgnores(models_new, m_model_ignores);

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
            updates_add = OptixUpdateAdded(models_new, diff.added);
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

        // shortcut
        rm::OptixInstancesPtr insts_old = 
            std::dynamic_pointer_cast<rm::OptixInstances>(
                m_map->scene()->getRoot());
        
        if(updates_add.size() > 0)
        {
            for(auto elem : updates_add)
            {
                rm::OptixInstPtr inst = elem.first;
                unsigned int geom_id = m_map->scene()->add(inst->geometry());
                unsigned int inst_id = insts_old->add(inst);
            }
            std::cout << "New map elements" << std::endl;
            std::cout << "- geometries: " << m_map->scene()->geometries().size() << std::endl;
            std::cout << "- instances: " << insts_old->instances().size() << std::endl;
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
            if(!meshes_to_update.empty())
            {
                // std::cout << "2. APPLY UPDATES" << std::endl;
                //     std::cout << "- Prepare instance updates " << meshes_to_update.size() << ": " << el << "s" << std::endl;
            }
        }

        if(diff.ModelRemoved())
        {
            for(auto model_id : diff.removed)
            {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
                    continue;
                }

                auto insts = m_model_meshes[model_id];

                for(auto inst : insts)
                {
                    insts_old->remove(inst);
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

            std::cout << "SCENE UPDATE: " << scene_changes << " changes" << std::endl;

            sw();
            insts_old->commit();
            m_map->scene()->commit();
            el = sw();
            std::cout << "- Scene update finished in " << el << "s" << std::endl;

            if(m_map_mutex)
            {
                m_map_mutex->unlock();
            }
        }
    }

    m_sensors_loaded = !m_scene_state.update(models_new, diff);

    if(!m_sensors_loaded)
    {
        std::cout << "Reload sensors!" << std::endl;
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
                std::cout << "[RmagineOptixMap] Found Rmagine spherical sensor " << spherical->ScopedName() << std::endl;
                spherical->setLock(m_map_mutex);
                spherical->setMap(m_map);
                if(!m_map_mutex)
                {
                    std::cout << "[RmagineOptixMap] no mutex " << std::endl;
                }
                
            }
        }
        m_sensors_loaded = true;
    }
}

void RmagineOptixMap::OnWorldUpdate(const common::UpdateInfo& info)
{
    // UpdateState();
    // UpdateSensors();

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

GZ_REGISTER_WORLD_PLUGIN(RmagineOptixMap)

} // namespace gazebo