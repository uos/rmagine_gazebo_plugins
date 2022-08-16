#ifndef RMAGINE_GAZEBO_HELPER_FUNCTIONS_H
#define RMAGINE_GAZEBO_HELPER_FUNCTIONS_H

#include <unordered_map>
#include <gazebo/physics/Model.hh>

namespace gazebo
{

std::unordered_map<uint32_t, physics::ModelPtr> ToIdMap(
        std::vector<physics::ModelPtr> models);

// void updateModelIgnores(
//     std::vector<physics::ModelPtr> models, 
//     std::unordered_set<uint32_t>& model_ignores);

void updateModelIgnores(
    std::unordered_map<uint32_t, physics::ModelPtr> models, 
    std::unordered_set<uint32_t>& model_ignores);

template<typename T>
inline std::unordered_set<T> get_union(
    const std::unordered_set<T>& a, const std::unordered_set<T>& b)
{
    std::unordered_set<T> res = a;
    res.insert(b.begin(), b.end());
    return res;
}


} // namespace gazebo

#endif // RMAGINE_GAZEBO_HELPER_FUNCTIONS_H