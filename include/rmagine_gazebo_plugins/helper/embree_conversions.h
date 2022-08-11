#ifndef RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H
#define RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H

#include <gazebo/msgs/msgs.hh>
#include <rmagine/map/embree/EmbreeScene.hpp>
#include <rmagine/map/embree/EmbreeGeometry.hpp>
#include <rmagine/map/AssimpIO.hpp>

#include <gazebo/common/URI.hh>
#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/HeightmapData.hh>

#include "conversions.h"



namespace gazebo
{

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::PlaneGeom& plane);

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::BoxGeom& box);

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::SphereGeom& sphere);

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::CylinderGeom& cylinder);

rmagine::EmbreeGeometryPtr to_rm_embree(const msgs::HeightmapGeom& heightmap);

// rmagine::EmbreeScenePtr to_rm_embree(const msgs::MeshGeom& gzmesh);

rmagine::EmbreeScenePtr to_rm_embree_gazebo(const msgs::MeshGeom& gzmesh);

rmagine::EmbreeScenePtr to_rm_embree_assimp(const msgs::MeshGeom& gzmesh);

} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H