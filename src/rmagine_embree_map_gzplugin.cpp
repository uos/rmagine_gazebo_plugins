#include <rmagine_gazebo_plugins/rmagine_embree_map_gzplugin.h>

#include <gazebo/sensors/SensorsIface.hh>


#include <gazebo/sensors/SensorManager.hh>


#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>


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
    m_map_mutex = std::make_shared<std::shared_mutex>();
    m_world = _world;
    m_sdf = _sdf;

    m_world_update_conn= event::Events::ConnectWorldUpdateBegin(
        std::bind(&RmagineEmbreeMap::OnWorldUpdate, this, std::placeholders::_1)
    );

    // create empty map
    m_map = std::make_shared<rm::EmbreeMap>();
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

std::unordered_set<uint32_t> RmagineEmbreeMap::ComputeTransformed(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    for(auto elem : models_new)
    {
        // 1. check pose change
        auto it_pose_old = m_poses.find(elem.first);

        if(it_pose_old != m_poses.end())
        {
            // also exists in old models poses
            // next: check if model pose was changed
            ignition::math::Pose3d pose_new = elem.second->RelativePose();
            ignition::math::Pose3d pose_old = it_pose_old->second;
            ignition::math::Pose3d pose_delta = pose_old.Inverse() * pose_new;

            if(pose_delta.Pos().Length() > m_changed_delta_trans)
            {
                // translation delta high enough
                ret.insert(elem.first);
            } else {
                ignition::math::Quaternion q = pose_delta.Rot();
                ignition::math::Vector3d vec(q.X(), q.Y(), q.Z());
                double angle = 2.0 * atan2(vec.Length(), q.W());

                if(angle > m_changed_delta_rot)
                {
                    // angle delta high enough
                    ret.insert(elem.first);
                }
            }
        }
    }

    return ret;
}

std::unordered_set<uint32_t> RmagineEmbreeMap::ComputeScaled(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_old,
    const std::unordered_map<uint32_t, physics::ModelPtr>& models_new) const
{
    std::unordered_set<uint32_t> ret;

    for(auto elem : models_new)
    {
        // 2. check scale change
        auto it_scale_old = m_scales.find(elem.first);

        if(it_scale_old != m_scales.end())
        {
            double scale_change = (it_scale_old->second - elem.second->Scale()).Length();
            if(scale_change > m_changed_delta_scale)
            {
                ret.insert(elem.first);
            }
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
    ret.transformed = ComputeTransformed(models_old, models_new);
    ret.scaled = ComputeScaled(models_old, models_new);

    return ret;
}


rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::PlaneGeom& plane) const
{
    msgs::Vector2d size = plane.size();
    // TODO: use normal
    msgs::Vector3d normal = plane.normal();

    std::vector<rm::Vector3> vertices;
    std::vector<rm::Face> faces;
    rm::genPlane(vertices, faces);

    rm::EmbreeMeshPtr mesh(new rm::EmbreeMesh(vertices.size(), faces.size()));

    std::copy(vertices.begin(), vertices.end(), mesh->vertices.raw());
    std::copy(faces.begin(), faces.end(), mesh->faces);

    rm::Vector3 scale;
    scale.x = size.x();
    scale.y = size.y();
    scale.z = 1.0;
    mesh->setScale(scale);

    return mesh;                            
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::BoxGeom& box) const
{
    msgs::Vector3d size = box.size();

    // fill embree mesh
    rm::EmbreeCubePtr mesh = std::make_shared<rm::EmbreeCube>();

    rm::Vector3 rm_scale = to_rm(size);
    mesh->setScale(rm_scale);

    return mesh;
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::SphereGeom& sphere) const
{
    rm::EmbreeSpherePtr mesh = std::make_shared<rm::EmbreeSphere>(30, 30);

    float diameter = sphere.radius() * 2.0;
    rm::Vector3 scale = {diameter, diameter, diameter};
    mesh->setScale(scale);

    return mesh;
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::CylinderGeom& cylinder) const
{
    std::vector<rm::Vector3> vertices;
    std::vector<rm::Face> faces;

    rm::genCylinder(vertices, faces, 100);

    rmagine::EmbreeMeshPtr mesh(new rmagine::EmbreeMesh(vertices.size(), faces.size()));

    std::copy(vertices.begin(), vertices.end(), mesh->vertices.raw());
    std::copy(faces.begin(), faces.end(), mesh->faces);

    float radius = cylinder.radius();
    float diameter = radius * 2.0;
    float height = cylinder.length();
    
    mesh->setScale({diameter, diameter, height});

    return mesh;
}

rmagine::EmbreeMeshPtr RmagineEmbreeMap::to_rmagine(
    const msgs::MeshGeom& gzmesh) const
{
    rmagine::EmbreeMeshPtr mesh;

    std::string filename = gzmesh.filename();
    msgs::Vector3d scale = gzmesh.scale();

    if(!common::exists(gzmesh.filename()))
    {
        filename = common::SystemPaths::Instance()->FindFileURI(gzmesh.filename());
    }

    std::cout << "Loading mesh from file " << filename << std::endl;

    Assimp::Importer importer;
    const aiScene* ascene = importer.ReadFile(filename, 0);

    if(!ascene)
    {
        std::cout << "WARNING could not load mesh from " << filename << std::endl;
        return mesh;
    }

    if(ascene->mNumMeshes > 1)
    {
        std::cout << "WARNING " << filename << " has more than one mesh. loading first" << std::endl;
    }

    if(ascene->mNumMeshes > 0)
    {
        const aiMesh* amesh = ascene->mMeshes[0];
        std::cout << amesh->mNumVertices << " vertices. " << amesh->mNumFaces << " faces." << std::endl;
        
        
        
        

        // ascene->mRootNode

        mesh = std::make_shared<rm::EmbreeMesh>(amesh);
        mesh->setScale(to_rm(scale));

        
        // { // debug
        //     const aiNode* root_node = ascene->mRootNode;

        //     std::cout << "instances: " << root_node->mNumChildren << std::endl;

        //     for(size_t child_id = 0; child_id < root_node->mNumChildren; child_id++)
        //     {
        //         std::cout << "Instance " << child_id << std::endl;
        //         const aiNode* n = root_node->mChildren[child_id];
        //         std::cout << "- name: " << n->mName.C_Str() << std::endl;
        //         std::cout << "- meshes: " << n->mNumMeshes << std::endl;
                
        //         rm::Matrix4x4 T;
        //         rm::convert(n->mTransformation, T);

        //         std::cout << "- transform: " << std::endl;
        //         std::cout << T << std::endl;

        //         std::cout << "- children: " << n->mNumChildren << std::endl;
        //         if(n->mNumChildren > 0)
        //         {
        //             // std::cout << "Has more children" << std::endl;
        //             for(size_t i=0; i<n->mNumChildren; i++)
        //             {
        //                 const aiNode* n2 = n->mChildren[i];
        //                 std::cout << "- Instance " << i << std::endl;
        //                 std::cout << "-- name: " << n2->mName.C_Str() << std::endl;
        //                 std::cout << "-- meshes: " << n2->mNumMeshes << std::endl;
        //                 rm::Matrix4x4 T2;
        //                 rm::convert(n2->mTransformation, T2);
        //                 std::cout << "-- transform: " << std::endl;
        //                 std::cout << T2 << std::endl;
        //                 std::cout << "-- children: " << n2->mNumChildren << std::endl;

        //                 rm::Matrix4x4 T_res = T * T2;
        //                 std::cout << "-- transform total: " << std::endl;
        //                 std::cout << T_res << std::endl;

        //                 rm::Vector3 scale = {T_res(0,0), T_res(1,1), T_res(2,2)};
        //                 scale = scale.mult_ewise(mesh->scale());
        //                 mesh->setScale(scale);
        //                 std::cout << "Set scale to " << scale << std::endl;
                        
        //             }
        //         }
        //     }
        // }

    }
    
    return mesh;
}

void RmagineEmbreeMap::UpdateState()
{
    // std::cout << "UpdateState" << std::endl;
    std::vector<physics::ModelPtr> models = m_world->Models();

    std::unordered_map<uint32_t, physics::ModelPtr> models_new = ToIdMap(models);

    // std::cout << "ComputeDiff" << std::endl;
    auto diff = ComputeDiff(m_models, models_new);

    // std::cout << "Apply changes to embree" << std::endl;

    // apply changes to rmagine
    if(diff.HasChanged())
    {
        // TODO! translate gazebo models to embree map instances
        std::cout << diff << std::endl;

        size_t scene_changes = 0;

        // Insert new meshes
        if(diff.ModelAdded())
        {
            std::cout << "1. ADD MODELS" << std::endl;
            for(auto model_id : diff.added)
            {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
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

                            rmagine::EmbreeMeshPtr mesh;
                            std::string mesh_type;

                            if(geom.has_mesh())
                            {
                                std::cout << "ADD MESH..." << std::endl;
                                msgs::MeshGeom gzmesh = geom.mesh();
                                mesh = to_rmagine(gzmesh);
                                mesh_type = "MESH";
                            }

                            if(geom.has_box())
                            {
                                std::cout << "ADD BOX!" << std::endl;
                                msgs::BoxGeom box = geom.box();
                                mesh = to_rmagine(box);
                                mesh_type = "BOX";
                            }

                            if(geom.has_cylinder())
                            {
                                std::cout << "ADD CYLINDER!" << std::endl;
                                msgs::CylinderGeom cylinder = geom.cylinder();
                                mesh = to_rmagine(cylinder);
                                mesh_type = "CYLINDER";
                            }

                            if(geom.has_sphere())
                            {
                                std::cout << "ADD SPHERE!" << std::endl;
                                msgs::SphereGeom sphere = geom.sphere();
                                mesh = to_rmagine(sphere);
                                mesh_type = "SPHERE";
                            }

                            if(geom.has_plane())
                            {
                                std::cout << "ADD PLANE!" << std::endl;
                                msgs::PlaneGeom plane = geom.plane();
                                mesh = to_rmagine(plane);
                                mesh_type = "PLANE";
                            }

                            if(geom.has_heightmap())
                            {
                                std::cout << "TODO: HEIGHTMAP" << std::endl;
                            }

                            if(mesh)
                            {   
                                mesh->setTransform(Tvw);
                                mesh->apply();
                                mesh->commit();

                                std::cout << "Add mesh" << std::endl;
                                unsigned int mesh_id = m_map->scene->add(mesh);
                                scene_changes++;
                                m_visual_to_mesh[key] = mesh_id;
                                

                                std::cout << "MESH created" << std::endl;
                                std::cout << "- type: " << mesh_type << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;
                                std::cout << "- scale: " << mesh->scale() << std::endl;
                                std::cout << "- transform: " << mesh->transform() << std::endl;

                                if(m_model_meshes.find(model_id) == m_model_meshes.end())
                                {
                                    m_model_meshes[model_id] = {};
                                }

                                m_model_meshes[model_id].push_back(mesh);
                            }
                        }
                    }
                }
            }

            // TODO: check 
            // error scene not committed sometimes when adding meshes. Could result in segfaults. 
            // std::cout << "1. models added." << std::endl;
        }
        
        if(diff.ModelChanged())
        {
            std::cout << "2. UPDATE SCENE - prepare" << std::endl;

            std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_transform;
            // Transform existing meshes
            if(diff.ModelTransformed())
            {
                std::cout << "2.1. APPLY TRANSFORMATIONS" << std::endl;
                rm::StopWatch sw;
                double el;

                sw();

                auto meshes = m_map->scene->geometries();

                for(auto model_id : diff.transformed)
                {
                    if(m_model_ignores.find(model_id) != m_model_ignores.end())
                    {
                        continue;
                    }

                    // get embree id of model
                    auto model = m_models[model_id];
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
                                std::cout << "Found visual to transform" << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;

                                ignition::math::Pose3d link_world_pose = link->WorldPose();
                                msgs::Pose vis_pose = vis.pose();

                                // convert to rmagine
                                rm::Transform Tlw = to_rm(link_world_pose);
                                rm::Transform Tvl = to_rm(vis_pose);

                                rm::Transform Tvw = Tlw * Tvl;
                                std::cout << "- transform: " << Tvw << std::endl;

                                rm::EmbreeGeometryPtr mesh = it->second;
                                mesh->setTransform(Tvw);

                                meshes_to_transform.insert(mesh);

                            } else {
                                std::cout << "WARNING: visual not in mesh set. But it should." << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;
                            }
                        }
                    }
                }

                el = sw();
                std::cout << "- Prepare meshes transforms: " << el << " ms" << std::endl;
            }

            std::unordered_set<rm::EmbreeGeometryPtr> meshes_to_scale;

            if(diff.ModelScaled())
            {
                std::cout << "2.2. APPLY SCALINGS" << std::endl;
                rm::StopWatch sw;
                double el;

                sw();

                auto meshes = m_map->scene->geometries();

                for(auto model_id : diff.scaled )
                {
                    if(m_model_ignores.find(model_id) != m_model_ignores.end())
                    {
                        continue;
                    }

                    // get embree id of model
                    auto model = m_models[model_id];
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
                                std::cout << "Found visual to scale" << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;

                                msgs::Vector3d vis_scale = vis.scale();

                                // convert to rmagine
                                rm::Vector Svl = to_rm(vis_scale);

                                std::cout << "- mesh scale old: " << it->second->scale() << std::endl;
                                std::cout << "- model scale: " << model_scale << std::endl;
                                std::cout << "- visual scale: " << Svl << std::endl;
                                // std::cout << "- transform: " << Tvw << std::endl;

                                rm::EmbreeGeometryPtr mesh = it->second;

                                if(vis.has_geometry())
                                {
                                    msgs::Geometry geom = vis.geometry();
                                    if(geom.has_box())
                                    {
                                        msgs::BoxGeom box = geom.box();
                                        auto box_size = box.size();
                                        rm::Vector3 box_scale = to_rm(box_size);

                                        std::cout << "- box scale: " << box_scale << std::endl;
                                        std::cout << "- box size: " << model_scale << std::endl;

                                        mesh->setScale(model_scale);
                                    }

                                    if(geom.has_sphere())
                                    {
                                        msgs::SphereGeom sphere = geom.sphere();
                                        float diameter = sphere.radius() * 2.0;
                                        rm::Vector3 sphere_scale = {diameter, diameter, diameter};

                                        std::cout << "- sphere scale: " << sphere_scale << std::endl;
                                        std::cout << "- sphere size: " << model_scale << std::endl;

                                        mesh->setScale(model_scale);
                                    }

                                    if(geom.has_cylinder())
                                    {
                                        msgs::CylinderGeom cylinder = geom.cylinder();

                                        float radius = cylinder.radius();
                                        float diameter = radius * 2.0;
                                        float height = cylinder.length();

                                        rm::Vector3 cylinder_scale = {diameter, diameter, height};

                                        std::cout << "- cylinder scale: " << cylinder_scale << std::endl;
                                        std::cout << "- cylinder size: " << model_scale << std::endl;
                                        mesh->setScale(model_scale);
                                    }
                                }

                                meshes_to_scale.insert(mesh);
                            } else {
                                std::cout << "WARNING: visual not in mesh set. But it should." << std::endl;
                                std::cout << "- id: " << mesh_id << std::endl;
                                std::cout << "- key: " << key << std::endl;
                            }
                        }
                    }
                }

                el = sw();
                std::cout << "- Prepare meshes scalings: " << el << " ms" << std::endl;
            }


            // mutex?
            rm::StopWatch sw;
            double el;

            auto meshes_to_update = get_union(meshes_to_transform, meshes_to_scale);

            sw();
            for(auto mesh_to_update : meshes_to_update)
            {
                mesh_to_update->apply();
                auto mesh = std::dynamic_pointer_cast<rm::EmbreeMesh>(mesh_to_update);
                if(mesh)
                {
                    mesh->markAsChanged();
                }
                
                mesh_to_update->commit();
                scene_changes++;
            }

            el = sw();
            std::cout << "- Mesh update: " << el << "s" << std::endl;
        }
        
        if(diff.ModelRemoved())
        {
            std::cout << "3. APPLY REMOVALS" << std::endl;
            for(auto model_id : diff.removed)
            {
                if(m_model_ignores.find(model_id) != m_model_ignores.end())
                {
                    continue;
                }

                auto meshes = m_model_meshes[model_id];
                std::cout << "Remove " << meshes.size() << " meshes" << std::endl;

                for(auto mesh : meshes)
                {
                    
                    mesh->parent.lock()->remove(mesh->id);
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
            m_map->scene->commit();
            
            el = sw();
            std::cout << "- Scene update finished in " << el << "s" << std::endl;
            
            if(m_map_mutex)
            {
                m_map_mutex->unlock();
            }
        }
    }

    // apply changes to gazebo state
    if(diff.HasChanged())
    {
        // change
        for(auto change_name : diff.changed())
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
        m_updater_thread = std::async(std::launch::async, [this] {
                UpdateState();
                UpdateSensors();
            });
    }
}

GZ_REGISTER_WORLD_PLUGIN(RmagineEmbreeMap)

} // namespace gazebo