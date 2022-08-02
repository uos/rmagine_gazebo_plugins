#include "rmagine_gazebo_plugins/helper/helper_functions.h"

namespace gazebo
{

std::unordered_map<uint32_t, physics::ModelPtr> ToIdMap(
        const std::vector<physics::ModelPtr>& models)
{
    std::unordered_map<uint32_t, physics::ModelPtr> ret;

    for(auto model : models)
    {
        if(model)
        {
            uint32_t model_id = model->GetId();
            ret[model_id] = model;
        }
    }

    return ret;
}

void updateModelIgnores(
    const std::vector<physics::ModelPtr>& models, 
    std::unordered_set<uint32_t>& model_ignores)
{
    for(auto model : models)
    {
        if(model)
        {
            uint32_t model_id = model->GetId();

            auto sdf = model->GetSDF();
            if(sdf->HasElement("rmagine_ignore"))
            {
                model_ignores.insert(model_id);
            } else {
                if(model_ignores.find(model_id) != model_ignores.end())
                {
                    // erase from ignores
                    model_ignores.erase(model_id);
                }
            }
        }
    }
}

void updateModelIgnores(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models, 
    std::unordered_set<uint32_t>& model_ignores)
{
    for(auto elem : models)
    {
        auto model = elem.second;
        if(model)
        {
            uint32_t model_id = model->GetId();

            auto sdf = model->GetSDF();
            if(sdf->HasElement("rmagine_ignore"))
            {
                model_ignores.insert(model_id);
            } else {
                if(model_ignores.find(model_id) != model_ignores.end())
                {
                    // erase from ignores
                    model_ignores.erase(model_id);
                }
            }
        }
    }
}

} // namespace gazebo