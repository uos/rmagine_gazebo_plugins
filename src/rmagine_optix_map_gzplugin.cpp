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
#include <rmagine/map/optix/OptixInstances.hpp>
#include <rmagine/map/optix/OptixInst.hpp>

using namespace std::placeholders;
using namespace boost::algorithm;
using namespace std::chrono_literals;

namespace rm = rmagine;

namespace gazebo
{

RmagineOptixMap::RmagineOptixMap()
{
    std::cout << "[RmagineOptixMap] Construct." << std::endl;
}

RmagineOptixMap::~RmagineOptixMap()
{
    std::cout << "[RmagineOptixMap] Destroy." << std::endl;
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
}

void RmagineOptixMap::UpdateState()
{
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
        // TODO
        std::cout << "UPDATE OPTIX SCENE" << std::endl;
        std::cout << diff << std::endl;

        size_t num_added = 0;

        for(auto model_id : diff.added)
        {
            if(m_model_ignores.find(model_id) != m_model_ignores.end())
            {
                continue;
            }

            auto model_it = models_new.find(model_id);
            if(model_it == models_new.end())
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

                        std::vector<rmagine::OptixGeometryPtr> geoms;

                        if(gzgeom.has_box())
                        {
                            std::cout << "ADD BOX!" << std::endl;
                            msgs::BoxGeom box = gzgeom.box();
                            geoms.push_back(to_rm(box));
                        }

                        if(gzgeom.has_cylinder())
                        {
                            std::cout << "ADD CYLINDER!" << std::endl;
                            msgs::CylinderGeom cylinder = gzgeom.cylinder();
                            geoms.push_back(to_rm(cylinder));
                        }

                        if(gzgeom.has_sphere())
                        {
                            std::cout << "ADD SPHERE!" << std::endl;
                            msgs::SphereGeom sphere = gzgeom.sphere();
                            geoms.push_back(to_rm(sphere));
                        }

                        if(gzgeom.has_plane())
                        {
                            std::cout << "ADD PLANE!" << std::endl;
                            msgs::PlaneGeom plane = gzgeom.plane();
                            geoms.push_back(to_rm(plane));
                        }

                        for(auto geom : geoms)
                        {
                            // Transform from instance to visual (or is it to world: TODO check)
                            auto Tiv = geom->transform();
                            auto Tiw = Tvw * Tiv;

                            // Set transform from instance to world
                            // geom->setTransform(Tiw);
                            // geom->apply();
                            // geom->commit();

                            rm::OptixInstancesPtr insts_new = std::dynamic_pointer_cast<rm::OptixInstances>(geom);

                            rm::OptixInstancesPtr insts_old = 
                                std::dynamic_pointer_cast<rm::OptixInstances>(
                                    m_map->scene()->getRoot());

                            if(insts_new)
                            {
                                // collection of geoms -> merge into existing
                            } else {
                                std::cout << "ADD GEOMETRY" << std::endl;
                                geom->apply();
                                geom->commit();
                                unsigned int geom_id = m_map->scene()->add(geom);
                                std::cout << "- id: " << geom_id << std::endl;

                                // instanciate geometry
                                rm::OptixInstPtr inst = std::make_shared<rm::OptixInst>();
                                inst->setGeometry(geom);
                                inst->setTransform(Tiw);
                                inst->apply();

                                unsigned int inst_id = insts_old->add(inst);
                                std::cout << "Instanciated geometry " << geom_id << " as Instance "<< inst_id << std::endl;
                                num_added++;
                            }

                            // require intermediate transformation for later movements
                            // geom_to_visual[geom] = {key, Tiv, model_id};
                        }
                    }
                }
            }
        }

        if(num_added > 0)
        {
            rm::OptixInstancesPtr insts_old = 
                                std::dynamic_pointer_cast<rm::OptixInstances>(
                                    m_map->scene()->getRoot());
            
            if(m_map_mutex)
            {
                m_map_mutex->lock();
            }

            insts_old->commit();
            m_map->scene()->commit();

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
                spherical->setMap(m_map);
                if(!m_map_mutex)
                {
                    std::cout << "[RmagineOptixMap] no mutex " << std::endl;
                }
                spherical->setLock(m_map_mutex);
            }
        }
        m_sensors_loaded = true;
    }
}

void RmagineOptixMap::OnWorldUpdate(const common::UpdateInfo& info)
{
    UpdateState();
    UpdateSensors();

    // if(!m_updater_thread.valid() 
    // || m_updater_thread.wait_for(0ms) == std::future_status::ready)
    // {
    //     // TODO: dont compute this twice!
    //     std::vector<physics::ModelPtr> models = m_world->Models();
    //     std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);
    //     updateModelIgnores(models_new, m_model_ignores);

    //     SceneDiff diff = m_scene_state.diff(models_new, 
    //         m_changed_delta_trans, 
    //         m_changed_delta_rot, 
    //         m_changed_delta_scale);

    //     if(diff.HasChanged())
    //     {
    //         m_updater_thread = std::async(std::launch::async, [this] {
    //                 UpdateState();
    //                 UpdateSensors();
    //             });
    //     }
    // }
}

GZ_REGISTER_WORLD_PLUGIN(RmagineOptixMap)

} // namespace gazebo