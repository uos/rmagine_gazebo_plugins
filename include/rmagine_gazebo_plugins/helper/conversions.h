#ifndef RMAGINE_GAZEBO_PLUGINS_CONVERSIONS_H
#define RMAGINE_GAZEBO_PLUGINS_CONVERSIONS_H

#include <rmagine/math/types.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{

inline rmagine::Transform to_rm(const ignition::math::Pose3d& pose)
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

static rmagine::Transform to_rm(const msgs::Pose& pose)
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

inline rmagine::Vector to_rm(const msgs::Vector3d& vec)
{
    rmagine::Vector v;
    v.x = vec.x();
    v.y = vec.y();
    v.z = vec.z();
    return v;
}

inline rmagine::Vector to_rm(const ignition::math::Vector3d& vec)
{
    rmagine::Vector v;
    v.x = vec.X();
    v.y = vec.Y();
    v.z = vec.Z();
    return v;
}


} // namespace gazebo

#endif // RMAGINE_GAZEBO_PLUGINS_CONVERSIONS_H