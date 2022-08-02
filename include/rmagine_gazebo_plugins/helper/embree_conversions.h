#ifndef RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H
#define RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H

#include <gazebo/msgs/msgs.hh>
#include <rmagine/map/embree/EmbreeScene.hpp>


namespace gazebo
{

rmagine::EmbreeGeometryPtr to_rm(const msgs::PlaneGeom& plane);

rmagine::EmbreeGeometryPtr to_rm(const msgs::BoxGeom& box);

rmagine::EmbreeGeometryPtr to_rm(const msgs::SphereGeom& sphere);

rmagine::EmbreeGeometryPtr to_rm(const msgs::CylinderGeom& cylinder);

rmagine::EmbreeGeometryPtr to_rm(const msgs::HeightmapGeom& heightmap);

rmagine::EmbreeScenePtr to_rm(const msgs::MeshGeom& gzmesh);

} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_EMBREE_CONVERSIONS_H