#ifndef RMAGINE_GAZEBO_PLUGINS_OPTIX_CONVERSIONS_H
#define RMAGINE_GAZEBO_PLUGINS_OPTIX_CONVERSIONS_H

#include <gazebo/msgs/msgs.hh>
#include <rmagine/map/optix/OptixScene.hpp>


namespace gazebo
{


rmagine::OptixGeometryPtr to_rm_optix(const msgs::PlaneGeom& plane);

// rmagine::OptixGeometryPtr to_rm(const msgs::BoxGeom& box);

rmagine::OptixGeometryPtr to_rm_optix(const msgs::BoxGeom& box);

rmagine::OptixGeometryPtr to_rm_optix(const msgs::SphereGeom& sphere);

rmagine::OptixGeometryPtr to_rm_optix(const msgs::CylinderGeom& cylinder);

rmagine::OptixGeometryPtr to_rm_optix(const msgs::HeightmapGeom& heightmap);

rmagine::OptixScenePtr to_rm_optix_gazebo(const msgs::MeshGeom& gzmesh);

rmagine::OptixScenePtr to_rm_optix_assimp(const msgs::MeshGeom& gzmesh);

} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_OPTIX_CONVERSIONS_H