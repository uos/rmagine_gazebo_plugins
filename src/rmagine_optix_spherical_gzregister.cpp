#include <gazebo/gazebo.hh>
#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>


namespace gazebo
{
  class RegisterRmagineOptixSphericalPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterRmagineOptixSphericalPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      sensors::RegisterRmagineOptixSpherical();
      printf("Loaded RmagineOptixSpherical!\n");
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterRmagineOptixSphericalPlugin)
}