#include "rmagine_gazebo_plugins/helper/SceneState.hpp"

namespace gazebo
{

SceneState::SceneState(
    const std::unordered_map<uint32_t, physics::ModelPtr>& models)
:m_models(models)
{

}

} // namespace gazebo