#include <rmagine_gazebo_plugins/rmagine_embree_map_gzplugin.h>

#include <gazebo/sensors/SensorsIface.hh>


#include <gazebo/sensors/SensorManager.hh>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/find_iterator.hpp>
#include <rmagine/util/synthetic.h>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <iostream>
#include <chrono>

using namespace std::placeholders;
using namespace boost::algorithm;
using namespace std::chrono_literals;


namespace rm = rmagine;

namespace gazebo
{


static rm::Transform to_rm(const ignition::math::Pose3d& pose)
{
    rmagine::Transform T;
    T.R.x = pose.Rot().X();
    T.R.y = pose.Rot().Y();
    T.R.z = pose.Rot().Z();
    T.R.w = pose.Rot().W();
    T.t.x = pose.Pos().X();
    T.t.y = pose.Pos().Y();
    T.t.z = pose.Pos().Z();
    return T;
}

static rm::Transform to_rm(const msgs::Pose& pose)
{
    rmagine::Transform T;
    T.R.x = pose.orientation().x();
    T.R.y = pose.orientation().y();
    T.R.z = pose.orientation().z();
    T.R.w = pose.orientation().w();
    T.t.x = pose.position().x();
    T.t.y = pose.position().y();
    T.t.z = pose.position().z();
    return T;
}

static rm::Vector to_rm(const msgs::Vector3d& vec)
{
    rm::Vector v;
    v.x = vec.x();
    v.y = vec.y();
    v.z = vec.z();
    return v;
}

static rm::Vector to_rm(const ignition::math::Vector3d& vec)
{
    rm::Vector v;
    v.x = vec.X();
    v.y = vec.Y();
    v.z = vec.Z();
    return v;
}

RmagineEmbreeMap::RmagineEmbreeMap()
{
    std::cout << "[RmagineEmbreeMap] Construct." << std::endl;
}

RmagineEmbreeMap::~RmagineEmbreeMap()
{
    std::cout << "[RmagineEmbreeMap] Destroy." << std::endl;
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
    // m_sphere_sim = std::make_shared<rmagine::SphereSimulatorEmbree>(m_map);
}

std::unordered_map<uint32_t, physics::ModelPtr> RmagineEmbreeMap::ToIdMap(
    const std::vector<physics::ModelPtr>& models)
{
    std::unordered_map<uint32_t, physics::ModelPtr> ret;

    for(auto model : models)
    {
        if(model)
        {
            uint32_t model_id = model->GetId();

            auto sdf = model->GetSDF();
            if(sdf->HasElement("rmagine_ignore"))
            {
                m_model_ignores.insert(model_id);
            } else {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
                    // erase from ignores
                    m_model_ignores.erase(model_id);
                }
            }

            ret[model_id] = model;
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


rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::PlaneGeom& plane) const
{
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

    mesh->vertices[3].x = -size.x();
    mesh->vertices[3].y = -size.y();
    mesh->vertices[3].z = 0.0;

    mesh->faces[0].v0 = 1;
    mesh->faces[0].v1 = 0;
    mesh->faces[0].v2 = 3;

    mesh->faces[1].v0 = 3;
    mesh->faces[1].v1 = 2;
    mesh->faces[1].v2 = 1;

    return mesh;                            
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::BoxGeom& box) const
{
    msgs::Vector3d size = box.size();
    

    // box is in center: -size.x()/2     +size.x()/2
    std::vector<rm::Vector3> vertices;
    std::vector<rm::Face> faces;

    rm::genCube(vertices, faces);

    // scale
    // for(size_t i=0; i<vertices.size(); i++)
    // {
    //     vertices[i].x *= size.x();
    //     vertices[i].y *= size.y();
    //     vertices[i].z *= size.z();
    // }

    // fill embree mesh
    rmagine::EmbreeMeshPtr mesh(new rmagine::EmbreeMesh(
        m_map->device, vertices.size(), faces.size()));

    std::copy(vertices.begin(), vertices.end(), mesh->vertices.raw());
    std::copy(faces.begin(), faces.end(), mesh->faces);

    rm::Vector3 rm_scale = to_rm(size);
    mesh->setScale(rm_scale);

    return mesh;
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::SphereGeom& sphere) const
{
    std::vector<rm::Vector3> vertices;
    std::vector<rm::Face> faces;
    rm::genSphere(vertices, faces, 30, 30);

    

    // for(size_t i=0; i<vertices.size(); i++)
    // {
    //     vertices[i].x *= diameter;
    //     vertices[i].y *= diameter;
    //     vertices[i].z *= diameter;
    // }
    
    // fill embree mesh
    rmagine::EmbreeMeshPtr mesh(new rmagine::EmbreeMesh(
        m_map->device, vertices.size(), faces.size()));

    std::copy(vertices.begin(), vertices.end(), mesh->vertices.raw());
    std::copy(faces.begin(), faces.end(), mesh->faces);

    float diameter = sphere.radius() * 2.0;
    rm::Vector3 scale = {diameter, diameter, diameter};
    mesh->setScale(scale);

    return mesh;
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::MeshGeom& gzmesh) const
{
    rmagine::EmbreeMeshPtr mesh;

    std::string filename = gzmesh.filename();
    msgs::Vector3d scale = gzmesh.scale();

    Assimp::Importer importer;
    const aiScene* ascene = importer.ReadFile( filename, 0);

    if(ascene->mNumMeshes > 1)
    {
        std::cout << "WARNING " << filename << " has more than one mesh. loading first" << std::endl;
    }

    if(ascene->mNumMeshes > 0)
    {
        const aiMesh* amesh = ascene->mMeshes[0];
        mesh = std::make_shared<rm::EmbreeMesh>(m_map->device, amesh);
    }
    
    return mesh;
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
        
        size_t scene_changes = 0;

        // Insert new meshes
        for(auto model_id : diff.added)
        {
            
            // std::cout << "Add model " << model_id << " to embree map" << std::endl;
            
            if(m_model_ignores.find(model_id) != m_model_ignores.end())
            {
                // std::cout << "IGNORE adding model " << model_id << std::endl;
                continue;
            }
            
            physics::ModelPtr model = models_new[model_id];
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
                    rm::Transform Tvl = to_rm(vis_pose);

                    rm::Transform Tvw = Tlw * Tvl;

                    std::string key = vis.name();

                    if(vis.has_geometry())
                    {
                        msgs::Geometry geom = vis.geometry();
                        if(geom.has_mesh())
                        {
                            msgs::MeshGeom gzmesh = geom.mesh();
                            rmagine::EmbreeMeshPtr mesh = to_rmagine(gzmesh);

                            if(mesh)
                            {   
                                mesh->setTransform(Tvw);
                                mesh->commit();
                                unsigned int mesh_id = m_map->scene->add(mesh);
                                scene_changes++;
                                m_visual_to_mesh[key] = mesh_id;
                                std::cout << "MESH created" << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;
                                std::cout << "- filename: " << gzmesh.filename() << std::endl;
                            } else {
                                std::cout << "could not load mesh" << std::endl;
                            }
                        }

                        if(geom.has_box())
                        {
                            msgs::BoxGeom box = geom.box();
                            rm::EmbreeMeshPtr mesh = to_rmagine(box);

                            auto scale = box.size();
                            rm::Vector3 scale_rm = to_rm(scale);

                            // TODO: how to merge with link scale Svl. Check if this is right
                            // mesh->setScale(scale_rm.mult_ewise(Svl));
                            mesh->setTransform(Tvw);
                            mesh->apply();

                            mesh->commit();
                            unsigned int mesh_id = m_map->scene->add(mesh);
                            scene_changes++;
                            m_visual_to_mesh[key] = mesh_id;

                            std::cout << "BOX created" << std::endl;
                            std::cout << "- id: " << mesh_id << std::endl;
                            std::cout << "- key: " << key << std::endl;
                        }

                        if(geom.has_cylinder())
                        {
                            std::cout << "TODO: CYLINDER" << std::endl;
                        }

                        if(geom.has_sphere())
                        {
                            msgs::SphereGeom sphere = geom.sphere();
                            rmagine::EmbreeMeshPtr mesh = to_rmagine(sphere);
                            
                            // mesh->setScale(Svl);
                            mesh->setTransform(Tvw);
                            mesh->apply();

                            mesh->commit();


                            unsigned int mesh_id = m_map->scene->add(mesh);
                            scene_changes++;

                            m_visual_to_mesh[key] = mesh_id;
                            std::cout << "SPHERE created" << std::endl;
                            std::cout << "- id: " << mesh_id << std::endl;
                            std::cout << "- key: " << key << std::endl; 
                        }

                        if(geom.has_plane())
                        {
                            msgs::PlaneGeom plane = geom.plane();
                            rmagine::EmbreeMeshPtr mesh = to_rmagine(plane);
                            
                            mesh->setScale(Svl);
                            mesh->setTransform(Tvw);
                            mesh->apply();

                            mesh->commit();
                            // transform

                            unsigned int mesh_id = m_map->scene->add(mesh);
                            scene_changes++;
                            m_visual_to_mesh[key] = mesh_id;

                            std::cout << "PLANE created" << std::endl;
                            std::cout << "- id: " << mesh_id << std::endl;
                            std::cout << "- key: " << key << std::endl;
                        }

                        if(geom.has_heightmap())
                        {
                            std::cout << "TODO: HEIGHTMAP" << std::endl;
                        }
                    }
                }
            }
        }

        std::vector<rm::EmbreeMeshPtr> meshes_to_update;

        // Update existing meshes
        if(diff.changed.size() > 0)
        {
            rm::StopWatch sw;
            double el;

            sw();

            for(auto model_id : diff.changed)
            {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
                    continue;
                }

                // get embree id of model
                auto model = m_models[model_id];
                std::string model_name = model->GetName();
                std::vector<physics::LinkPtr> links = model->GetLinks();

                auto meshes = m_map->scene->meshes();

                ignition::math::Vector3d model_scale_gz = model->Scale();
                rm::Vector3 model_scale = to_rm(model_scale_gz);

                for(physics::LinkPtr link : links)
                {
                    std::map<uint32_t, msgs::Visual> visuals = link->Visuals();

                    for(auto elem : visuals)
                    {
                        msgs::Visual vis = elem.second;
                        std::string key = vis.name();
                        
                        auto mesh_vis_it = m_visual_to_mesh.find(key);

                        if(mesh_vis_it == m_visual_to_mesh.end())
                        {
                            std::cout << "WARNING mesh to update not found in embree. Skipping." << std::endl;
                            std::cout << "- key: " << key << std::endl;
                            continue;
                        }

                        unsigned int mesh_id = mesh_vis_it->second;
                        auto it = meshes.find(mesh_id);

                        if(it != meshes.end())
                        {
                            std::cout << "Found visual to update" << std::endl;
                            std::cout << "- id: " << mesh_id << std::endl;
                            std::cout << "- key: " << key << std::endl;

                            ignition::math::Pose3d link_world_pose = link->WorldPose();
                            msgs::Pose vis_pose = vis.pose();
                            msgs::Vector3d vis_scale = vis.scale();

                            // convert to rmagine
                            rm::Transform Tlw = to_rm(link_world_pose);
                            rm::Transform Tvl = to_rm(vis_pose);
                            rm::Vector Svl = to_rm(vis_scale);

                            rm::Transform Tvw = Tlw * Tvl;

                            std::cout << "- model scale: " << model_scale << std::endl;
                            std::cout << "- visual scale: " << Svl << std::endl;
                            std::cout << "- transform: " << Tvw << std::endl;

                            rm::EmbreeMeshPtr mesh = it->second;
                            
                            mesh->setScale(Svl);
                            mesh->setTransform(Tvw);

                            if(vis.has_geometry())
                            {
                                msgs::Geometry geom = vis.geometry();
                                if(geom.has_box())
                                {
                                    // msgs::BoxGeom box = geom.box();
                                    // auto geom_size = box.size();
                                    // rm::Vector3 scale_rm = to_rm(geom_size);
                                    mesh->setScale(model_scale);
                                    std::cout << "- box size: " << model_scale << std::endl;
                                }

                                if(geom.has_sphere())
                                {
                                    // msgs::BoxGeom box = geom.box();
                                    mesh->setScale(model_scale);
                                    std::cout << "- sphere size: " << model_scale << std::endl;
                                }
                            }

                            mesh->apply();
                            meshes_to_update.push_back(mesh);

                        } else {
                            std::cout << "WARNING: visual not in mesh set. But it should." << std::endl;
                            std::cout << "- id: " << mesh_id << std::endl;
                            std::cout << "- key: " << key << std::endl;
                        }
                    }
                }
            }

            el = sw();
            std::cout << "- Prepare meshes updates: " << el << " ms" << std::endl;
        }

        // mutex?
        rm::StopWatch sw;
        double el;

        sw();
        for(auto mesh_to_update : meshes_to_update)
        {
            mesh_to_update->markAsChanged();
            mesh_to_update->commit();
            scene_changes++;
        }
        el = sw();
        std::cout << "- Mesh update: " << el << "s" << std::endl;
        

        sw();
        if(scene_changes > 0)
        {
            m_map->scene->commit();
        }
        el = sw();
        std::cout << "- Scene update: " << el << "s" << std::endl;
    }

    // apply changes to gazebo state
    if(diff.HasChanged())
    {
        // change
        for(auto change_name : diff.changed)
        {
            auto model = m_models[change_name];

            m_poses[change_name] = model->RelativePose();
            m_scales[change_name] = model->Scale();
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
        m_updater_thread = std::async(std::launch::async, [this] {
                UpdateState();
                UpdateSensors();
            });
    }
}

GZ_REGISTER_WORLD_PLUGIN(RmagineEmbreeMap)

} // namespace gazebo