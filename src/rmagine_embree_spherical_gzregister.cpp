#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>


namespace gazebo
{
  class RegisterRmagineEmbreeSphericalPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterRmagineEmbreeSphericalPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      sensors::RegisterRmagineEmbreeSpherical();
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterRmagineEmbreeSphericalPlugin)
}