#include "rmagine_gazebo_plugins/helper/optix_conversions.h"

#include "rmagine_gazebo_plugins/helper/conversions.h"

#include <rmagine/math/types.h>
#include <rmagine/map/AssimpIO.hpp>
#include <rmagine/map/optix/optix_shapes.h>
#include <rmagine/util/prints.h>

#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/HeightmapData.hh>

#include <iostream>


#include <rmagine/map/optix/OptixInstances.hpp>
#include <rmagine/map/optix/OptixInst.hpp>

// #include <chrono>
// #include <thread>

// using namespace std::chrono_literals;



namespace rm = rmagine;

namespace gazebo
{

rmagine::OptixGeometryPtr to_rm(const msgs::PlaneGeom& plane)
{
    msgs::Vector2d size = plane.size();
    // TODO: use normal
    msgs::Vector3d normal = plane.normal();
    // rotation of normal? angle shortest path or so

    rm::OptixPlanePtr mesh = std::make_shared<rm::OptixPlane>();

    rm::Vector3 scale;
    scale.x = size.x();
    scale.y = size.y();
    scale.z = 1.0;

    mesh->setScale(scale);

    return mesh;
}

// rmagine::OptixGeometryPtr to_rm(const msgs::BoxGeom& box)
// {
//     // fill embree mesh
//     rm::OptixCubePtr mesh = std::make_shared<rm::OptixCube>();

//     msgs::Vector3d size = box.size();
//     rm::Vector3 rm_scale = to_rm(size);
//     mesh->setScale(rm_scale);

//     return mesh;
// }

rmagine::OptixInstPtr to_rm(const msgs::BoxGeom& box)
{
    // make basic mesh
    rm::OptixCubePtr mesh = std::make_shared<rm::OptixCube>();
    mesh->apply();
    mesh->commit();

    // make inst
    rm::OptixInstPtr mesh_inst = std::make_shared<rm::OptixInst>();
    mesh_inst->setGeometry(mesh);
    msgs::Vector3d size = box.size();
    rm::Vector3 rm_scale = to_rm(size);
    mesh_inst->setScale(rm_scale);
    mesh_inst->apply();

    return mesh_inst;
}

rmagine::OptixGeometryPtr to_rm(const msgs::SphereGeom& sphere)
{
    rm::OptixSpherePtr mesh = std::make_shared<rm::OptixSphere>(30, 30);

    float diameter = sphere.radius() * 2.0;
    rm::Vector3 scale = {diameter, diameter, diameter};
    mesh->setScale(scale);

    return mesh;
}

rmagine::OptixGeometryPtr to_rm(const msgs::CylinderGeom& cylinder)
{
    rm::OptixCylinderPtr mesh = std::make_shared<rm::OptixCylinder>(100);
    float radius = cylinder.radius();
    float diameter = radius * 2.0;
    float height = cylinder.length();
    
    mesh->setScale({diameter, diameter, height});

    return mesh;
}

rmagine::OptixGeometryPtr to_rm(const msgs::HeightmapGeom& heightmap)
{
    rmagine::OptixGeometryPtr ret;

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
        std::cout << "Heightmap data loaded." << std::endl;

        std::cout << "Data Info: " << std::endl;
        std::cout << "- height: " << data->GetHeight() << std::endl;
        std::cout << "- width: " << data->GetWidth() << std::endl;
        std::cout << "- max_elevation: " << data->GetMaxElevation() << std::endl;


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

        std::cout << "Loaded " << elevations.size() << " elevations." << std::endl;
        
        rm::OptixMeshPtr mesh = std::make_shared<rm::OptixMesh>();

        rm::Memory<rm::Vector3, rm::RAM> vertices(data->GetWidth() * data->GetHeight());

        rm::Memory<rm::Face, rm::RAM> faces(2 * (data->GetWidth() - 1) * (data->GetHeight() - 1) );

        rm::Vector3 offset = to_rm(heightmap.origin());

        float half_width = size.X() / 2.0;
        float half_height = size.Y() / 2.0;

        std::cout << "Filling " << vertices.size() << " vertices..." << std::endl;
        
        rm::Vector3 correction = {0.0, 0.0, 0.0};

        int center_vert = vertSize / 2;

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

                float xworld = static_cast<float>(xgrid) * scale_x;
                float yworld = static_cast<float>(ygrid) * scale_y;

                float zworld = elevations[buff_id];

                rm::Vector3 vertex_corrected = {
                    xworld,
                    yworld,
                    zworld
                };

                vertex_corrected += offset;

                assert(buff_id < mesh->vertices.size());

                vertices[buff_id] = vertex_corrected;
            }
        }

        std::cout << "Filling " << faces.size() << " faces..." << std::endl;

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

                // connect as in plane example
                faces[f0] = {v1, v0, v3};
                faces[f1] = {v3, v2, v1};
            }
        }

        // std::cout << "SLeep" << std::endl;
        // std::this_thread::sleep_for(10s);

        // std::cout << "Done." << std::endl;

        // std::cout << "Upload to GPU" << std::endl;
        mesh->vertices = vertices;
        mesh->faces = faces;
        
        // std::cout << "Compute Face Normals" << std::endl;

        mesh->computeFaceNormals();
        // std::cout << "Building ACC" << std::endl;

        mesh->apply();
        mesh->commit();

        // std::cout << "Done." << std::endl;

        ret = mesh;
        delete data;
    }

    return ret;
}

rm::OptixScenePtr to_rm(
    const msgs::MeshGeom& gzmesh)
{
    rm::OptixScenePtr ret;
    std::string filename = gzmesh.filename();

    if(!common::exists(gzmesh.filename()))
    {
        filename = common::SystemPaths::Instance()->FindFileURI(gzmesh.filename());
    }

    std::cout << "Loading mesh from file " << filename << std::endl;

    rm::AssimpIO io;
    const aiScene* ascene = io.ReadFile(filename, 0);
    if(!ascene)
    {
        std::cout << "WARNING could not load mesh from " << filename << std::endl;
        std::cerr << io.Importer::GetErrorString() << std::endl;
        return ret;
    }

    if(ascene->mNumMeshes > 0)
    {
        ret = rm::make_optix_scene(ascene);
    }
    
    return ret;
}

} // namespace gazebo