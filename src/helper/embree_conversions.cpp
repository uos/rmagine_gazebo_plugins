#include "rmagine_gazebo_plugins/helper/embree_conversions.h"

#include "rmagine_gazebo_plugins/helper/conversions.h"

#include <rmagine/math/types.h>
#include <rmagine/map/AssimpIO.hpp>
#include <rmagine/map/embree/embree_shapes.h>

#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/HeightmapData.hh>

#include <iostream>


namespace rm = rmagine;

namespace gazebo
{

rmagine::EmbreeGeometryPtr to_rm(const msgs::PlaneGeom& plane)
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

rmagine::EmbreeGeometryPtr to_rm(const msgs::BoxGeom& box)
{
    msgs::Vector3d size = box.size();

    // fill embree mesh
    rm::EmbreeCubePtr mesh = std::make_shared<rm::EmbreeCube>();

    rm::Vector3 rm_scale = to_rm(size);
    mesh->setScale(rm_scale);

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm(const msgs::SphereGeom& sphere)
{
    rm::EmbreeSpherePtr mesh = std::make_shared<rm::EmbreeSphere>(30, 30);

    float diameter = sphere.radius() * 2.0;
    rm::Vector3 scale = {diameter, diameter, diameter};
    mesh->setScale(scale);

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm(const msgs::CylinderGeom& cylinder)
{
    rm::EmbreeCylinderPtr mesh = std::make_shared<rm::EmbreeCylinder>(100);
    float radius = cylinder.radius();
    float diameter = radius * 2.0;
    float height = cylinder.length();
    
    mesh->setScale({diameter, diameter, height});

    return mesh;
}

rmagine::EmbreeGeometryPtr to_rm(const msgs::HeightmapGeom& heightmap)
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

        std::cout << "Filling " << mesh->vertices.size() << " vertices..." << std::endl;
        
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

                assert(buff_id < mesh->vertices.size());
                mesh->vertices[buff_id] = vertex_corrected;
            }
        }

        std::cout << "Filling " << mesh->Nfaces << " faces..." << std::endl;
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

                assert(f0 < mesh->Nfaces);
                assert(f1 < mesh->Nfaces);

                // connect as in plane example
                mesh->faces[f0] = {v1, v0, v3};
                mesh->faces[f1] = {v3, v2, v1};
            }
        }

        mesh->computeFaceNormals();

        ret = mesh;
        delete data;

    } else {
        std::cout << "Could not load " << filename << " with common::HeightmapDataLoader" << std::endl;
    }

    return ret;
}

rmagine::EmbreeScenePtr to_rm(const msgs::MeshGeom& gzmesh)
{
    rmagine::EmbreeScenePtr scene;
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
        return scene;
    }

    if(ascene->mNumMeshes > 0)
    {
        scene = rm::make_embree_scene(ascene);
        // scale every geometry?
        rm::Vector3 scale = to_rm(gzmesh.scale());
        for(auto elem : scene->geometries())
        {
            auto geom = elem.second;   
            geom->setScale(geom->scale().mult_ewise(scale));
        }
    }
    
    return scene;
}

} // namespace gazebo