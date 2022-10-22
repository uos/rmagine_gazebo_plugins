#include "rmagine_gazebo_plugins/helper/embree_conversions.h"

#include "rmagine_gazebo_plugins/helper/conversions.h"

#include <rmagine/math/types.h>
#include <rmagine/map/AssimpIO.hpp>
#include <rmagine/map/embree/embree_shapes.h>
#include <rmagine/util/prints.h>


#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/HeightmapData.hh>

#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Mesh.hh>

#include <gazebo/common/Console.hh>

#include <iostream>


namespace rm = rmagine;

namespace gazebo
{

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::PlaneGeom& plane)
{
    msgs::Vector2d size = plane.size();
    // TODO: use normal
    msgs::Vector3d normal = plane.normal();
    // rotation of normal? angle shortest path or so

    rm::EmbreePlanePtr mesh = std::make_shared<rm::EmbreePlane>();
    rm::Vector3 scale;
    scale.x = size.x();
    scale.y = size.y();
    scale.z = 1.0;
    mesh->setScale(scale);

    return mesh;   
}

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::BoxGeom& box)
{
    msgs::Vector3d size = box.size();

    // fill embree mesh
    rm::EmbreeCubePtr mesh = std::make_shared<rm::EmbreeCube>();

    rm::Vector3 rm_scale = to_rm(size);
    mesh->setScale(rm_scale);

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::SphereGeom& sphere)
{
    rm::EmbreeSpherePtr mesh = std::make_shared<rm::EmbreeSphere>(30, 30);

    float diameter = sphere.radius() * 2.0;
    rm::Vector3 scale = {diameter, diameter, diameter};
    mesh->setScale(scale);

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::CylinderGeom& cylinder)
{
    rm::EmbreeCylinderPtr mesh = std::make_shared<rm::EmbreeCylinder>(100);
    float radius = cylinder.radius();
    float diameter = radius * 2.0;
    float height = cylinder.length();
    
    mesh->setScale({diameter, diameter, height});

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::HeightmapGeom& heightmap)
{
    rmagine::EmbreeGeometryPtr ret;

    // print(heightmap);
    
    rm::Vector3 size = to_rm(heightmap.size());
    rm::Vector3 orig = to_rm(heightmap.origin());


    std::string filename = heightmap.filename();
    if(!common::exists(filename))
    {
        filename = common::SystemPaths::Instance()->FindFileURI(filename);
    }

    common::HeightmapData* data 
        = common::HeightmapDataLoader::LoadTerrainFile(filename);

    if(data)
    {
        gzdbg << "Heightmap data loaded." << std::endl;

        gzdbg << "Data Info: " << std::endl;
        gzdbg << "- height: " << data->GetHeight() << std::endl;
        gzdbg << "- width: " << data->GetWidth() << std::endl;
        gzdbg << "- max_elevation: " << data->GetMaxElevation() << std::endl;

        std::vector<float> elevations;
        // fill heights
        int subsampling = 1; // multiplies the resulution if set
        unsigned int vertSize = (data->GetWidth() * subsampling) - subsampling + 1;

        float scale_x = heightmap.size().x() / (vertSize - 1);
        float scale_y = heightmap.size().y() / (vertSize - 1);

        ignition::math::Vector3d size, scale;
        size.X() = heightmap.size().x();
        size.Y() = heightmap.size().y();
        size.Z() = heightmap.size().z();

        // scale. or: size of one pixel
        scale.X(size.X() / vertSize);
        scale.Y(size.Y() / vertSize);
        if(ignition::math::equal(data->GetMaxElevation(), 0.0f)) 
        {
            scale.Z(fabs(size.Z()));
        } else {
            scale.Z(fabs(size.Z()) / data->GetMaxElevation());
        }

        bool flipY = true;
        data->FillHeightMap(subsampling, vertSize, size, scale, flipY, elevations);

        gzdbg << "Loaded " << elevations.size() << " elevations." << std::endl;
        
        // NEXT: make mesh
        // - we need one vertex per elevation
        // - we need 1 quad for 4 vertices neighbors
        //   -> (W-1)*(H-1) quads
        //   -> (W-1)*(H-1)*2 triangles

        rm::EmbreeMeshPtr mesh = std::make_shared<rm::EmbreeMesh>(
            data->GetWidth() * data->GetHeight(), 
            2 * (data->GetWidth() - 1) * (data->GetHeight() - 1) 
        );

        rm::Vector3 offset = to_rm(heightmap.origin());

        float half_width = size.X() / 2.0;
        float half_height = size.Y() / 2.0;


        
        
        rm::Vector3 correction = {0.0, 0.0, 0.0};

        int center_vert = vertSize / 2;
        
        auto mesh_vertices = mesh->vertices();

        gzdbg << "Filling " << mesh_vertices.size() << " vertices..." << std::endl;

        // the following cannot handle higher subsampling levels yet
        for(size_t yimg=0; yimg < vertSize; yimg++)
        {
            for(size_t ximg=0; ximg < vertSize; ximg++)
            {
                // in grid coords
                size_t buff_id = yimg * vertSize + ximg;
                float height = elevations[buff_id];
                
                // grid origin is (0,0) at (ximg, yimg) = (vertSize/2, vertSize/2)
                // example 5x5 img, image id (2,2) is grid id (0,0)
                int xgrid = static_cast<int>(ximg) - center_vert;
                int ygrid = static_cast<int>(yimg) - center_vert;

                // std::cout << "(" << ximg << ", " << yimg << ")i -> (" << xgrid << ", " << ygrid << ")g" << std::endl;

                float xworld = static_cast<float>(xgrid) * scale_x;
                float yworld = static_cast<float>(ygrid) * scale_y;

                // std::cout << "(" << xgrid << ", " << ygrid << ")g -> (" << xworld << ", " << yworld << ")w" << std::endl;

                float zworld = elevations[buff_id];

                rm::Vector3 vertex_corrected = {
                    xworld,
                    yworld,
                    zworld
                };

                vertex_corrected += offset;

                // assert(buff_id < mesh->vertices.size());
                mesh_vertices[buff_id] = vertex_corrected;
            }
        }

        auto mesh_faces = mesh->faces();

        gzdbg << "Filling " << mesh_faces.size() << " faces..." << std::endl;
        // fill faces
        for(size_t i=1; i<vertSize; i++)
        {
            for(size_t j=1; j<vertSize; j++)
            {
                unsigned int v0 = (i-1) * vertSize + (j-1);
                unsigned int v1 = (i) * vertSize + (j-1);
                unsigned int v2 = (i) * vertSize +   (j);
                unsigned int v3 = (i-1) * vertSize +   (j);

                unsigned int quad_id = (i-1) * (vertSize - 1) + (j-1);

                unsigned int f0 = quad_id * 2 + 0;
                unsigned int f1 = quad_id * 2 + 1;

                // assert(f0 < mesh->Nfaces);
                // assert(f1 < mesh->Nfaces);

                // connect as in plane example
                mesh_faces[f0] = {v1, v0, v3};
                mesh_faces[f1] = {v3, v2, v1};
            }
        }

        mesh->computeFaceNormals();

        ret = mesh;
        delete data;

    } else {
        gzwarn << "Could not load " << filename << " with common::HeightmapDataLoader" << std::endl;
    }

    return ret;
}

rmagine::EmbreeScenePtr to_rm_embree_assimp(const msgs::MeshGeom& gzmesh)
{
    rmagine::EmbreeScenePtr scene;
    std::string filename = gzmesh.filename();

    if(!common::exists(gzmesh.filename()))
    {
        filename = common::SystemPaths::Instance()->FindFileURI(gzmesh.filename());
    }

    gzdbg << "Assimp Loader: Loading mesh from file " << filename << std::endl;

    rm::AssimpIO io;
    const aiScene* ascene = io.ReadFile(filename, 0);
    if(!ascene)
    {
        gzwarn << "WARNING could not load mesh from " << filename << std::endl;
        gzwarn << io.Importer::GetErrorString() << std::endl;
        return scene;
    }

    if(ascene->mNumMeshes > 0)
    {
        scene = rm::make_embree_scene(ascene);

        if(scene)
        {
            // scale every geometry?
            rm::Vector3 scale = to_rm(gzmesh.scale());
            for(auto elem : scene->geometries())
            {
                auto geom = elem.second;   
                geom->setScale(geom->scale().multEwise(scale));
            }
        } else {
            gzwarn << "WARNING Assimp Import: make_embree_scene failed." << std::endl;
        }
    } else {
        gzwarn << "WARNING Assimp Import: number of meshes == 0" << std::endl;
    }
    
    return scene;
}

rm::EmbreeScenePtr to_rm_embree(
    const common::Mesh* gzmesh)
{
    rm::EmbreeScenePtr ret = std::make_shared<rm::EmbreeScene>();

    gzdbg << "GAZEBO mesh loaded: " << std::endl;
    gzdbg << "- name: " << gzmesh->GetName() << std::endl;
    gzdbg << "- vertices: " << gzmesh->GetVertexCount() << std::endl;
    gzdbg << "- normals: " << gzmesh->GetNormalCount() << std::endl;
    gzdbg << "- indices: " << gzmesh->GetIndexCount() << std::endl;
    gzdbg << "- min, max: " << to_rm(gzmesh->Min()) << ", " << to_rm(gzmesh->Max()) << std::endl;
    gzdbg << "- materials: " << gzmesh->GetMaterialCount() << std::endl;
    gzdbg << "- sub meshes: " << gzmesh->GetSubMeshCount() << std::endl;


    for(size_t i = 0; i < gzmesh->GetSubMeshCount(); i++)
    {
        const common::SubMesh* gzsubmesh = gzmesh->GetSubMesh(i);
        gzdbg << "SUBMESH " << i << std::endl;
        gzdbg << "-- name: " << gzsubmesh->GetName() << std::endl; 
        gzdbg << "-- vertices: " << gzsubmesh->GetVertexCount() << std::endl;
        gzdbg << "-- normals: " << gzsubmesh->GetNormalCount() << std::endl;
        gzdbg << "-- indices: " << gzsubmesh->GetIndexCount() << std::endl;
        gzdbg << "-- faces (indices/3): " << gzsubmesh->GetIndexCount() / 3 << std::endl;
        gzdbg << "-- tex coords: " << gzsubmesh->GetTexCoordCount() << std::endl;
        gzdbg << "-- node assignments: " << gzsubmesh->GetNodeAssignmentsCount() << std::endl;
        gzdbg << "-- mat index: " << gzsubmesh->GetMaterialIndex() << std::endl;
        gzdbg << "-- min, max: " << to_rm(gzsubmesh->Min()) << ", " << to_rm(gzsubmesh->Max()) << std::endl;
    
        if(gzsubmesh->GetPrimitiveType() == common::SubMesh::PrimitiveType::TRIANGLES)
        {

            rm::EmbreeMeshPtr mesh = std::make_shared<rm::EmbreeMesh>(
                gzsubmesh->GetVertexCount(), 
                gzsubmesh->GetIndexCount() / 3);

            // TODO fill
            
            gzdbg << "Converting vertices" << std::endl;
            auto mesh_vertices = mesh->vertices();
            for(size_t i=0; i<mesh_vertices.size(); i++)
            {
                mesh_vertices[i] = to_rm(gzsubmesh->Vertex(i));
            }

            gzdbg << "Converting faces" << std::endl;
            auto mesh_faces = mesh->faces();
            for(size_t i=0; i<mesh_faces.size(); i++)
            {
                mesh_faces[i] = {
                    gzsubmesh->GetIndex(i * 3 + 0),
                    gzsubmesh->GetIndex(i * 3 + 1),
                    gzsubmesh->GetIndex(i * 3 + 2)
                };
            }

            // COPY VERTEX NORMALS IF AVAILABLE
            if(gzsubmesh->GetNormalCount())
            {
                gzdbg << "Converting vertex normals" << std::endl;
                mesh->initVertexNormals();

                auto mesh_vertex_normals = mesh->vertexNormals();
                
                for(size_t i=0; i<mesh_vertex_normals.size(); i++)
                {
                    mesh_vertex_normals[i] = to_rm(gzsubmesh->Normal(i));
                }
            }

            gzdbg << "Compute face normals" << std::endl;
            mesh->computeFaceNormals();

            gzdbg << "Build acceleration structures" << std::endl;
            mesh->apply();
            mesh->commit();

            gzdbg << "Add to scene" << std::endl;
            ret->add(mesh);
        } else {
            static std::unordered_map<common::SubMesh::PrimitiveType, std::string> prim_strings {
                { common::SubMesh::PrimitiveType::POINTS, "POINTS" },
                { common::SubMesh::PrimitiveType::LINES, "LINES" },
                { common::SubMesh::PrimitiveType::LINESTRIPS, "LINESTRIPS" },
                { common::SubMesh::PrimitiveType::TRIANGLES, "TRIANGLES" },
                { common::SubMesh::PrimitiveType::TRIFANS, "TRIFANS" },
                { common::SubMesh::PrimitiveType::TRISTRIPS, "TRISTRIPS" },
            };

            gzwarn << "Mesh to Embree - Primitive type not implemented: [" << prim_strings[gzsubmesh->GetPrimitiveType()] << "]" << std::endl;
        }
    }

    return ret;
}

rmagine::EmbreeScenePtr to_rm_embree_gazebo(const msgs::MeshGeom& gzmesh)
{
    gzdbg << "load mesh with gazebo " << std::endl;
    rmagine::EmbreeScenePtr ret;

    std::string filename = gzmesh.filename();

    if(!common::exists(gzmesh.filename()))
    {
        filename = common::SystemPaths::Instance()->FindFileURI(gzmesh.filename());
    }

    gzdbg << "Gazebo Import: Loading mesh from file " << filename << std::endl;

    common::MeshManager *meshManager = common::MeshManager::Instance();

    const common::Mesh* mesh = meshManager->GetMesh(filename);

    if(!mesh)
    {
        mesh = meshManager->Load(filename);
        if(!mesh)
        {
            gzerr << "Gazebo Import: MeshManager failed to GetMesh and Load. Mesh file [" << filename << "]" << std::endl;
            return ret;
        }
    }

    // mesh is set
    ret = to_rm_embree(mesh);

    if(ret)
    {
        // scale every geometry?
        rm::Vector3 scale = to_rm(gzmesh.scale());
        for(auto elem : ret->geometries())
        {
            auto geom = elem.second;   
            geom->setScale(geom->scale().multEwise(scale));
            geom->apply();
        }
    } else {
        gzwarn << "WARNING Gazebo Import: to_rm_embree(common::Mesh*) failed." << std::endl;
    }

    return ret;
}

} // namespace gazebo