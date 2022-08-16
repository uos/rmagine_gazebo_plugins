#include "rmagine_gazebo_plugins/helper/helper_functions.h"

#include <unordered_map>

namespace gazebo
{

std::unordered_map<uint32_t, physics::ModelPtr> ToIdMap(
        std::vector<physics::ModelPtr> models)
{
    std::unordered_map<uint32_t, physics::ModelPtr> ret;


    for(size_t i=0; i<models.size() && i<models.capacity(); i++)
    {
        physics::ModelPtr model = models[i];
        if(model)
        {
            uint32_t model_id = model->GetId();
            ret[model_id] = model;
        }
    }

    // for(physics::ModelPtr model : models)
    // {
    //     if(model)
    //     {
    //         uint32_t model_id = model->GetId();
    //         ret[model_id] = model;
    //     }
    // }
    // std::cout << "ToIdMap done." << std::endl;


    return ret;
}

// void updateModelIgnores(
//     const std::vector<physics::ModelPtr>& models, 
//     std::unordered_set<uint32_t>& model_ignores)
// {
//     for(auto model : models)
//     {
//         if(model)
//         {
//             uint32_t model_id = model->GetId();

//             auto sdf = model->GetSDF();
//             if(sdf->HasElement("rmagine_ignore"))
//             {
//                 model_ignores.insert(model_id);
//             } else {
//                 if(model_ignores.find(model_id) != model_ignores.end())
//                 {
//                     // erase from ignores
//                     model_ignores.erase(model_id);
//                 }
//             }
//         }
//     }
// }

void updateModelIgnores(
    std::unordered_map<uint32_t, physics::ModelPtr> models, 
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