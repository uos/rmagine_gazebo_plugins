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