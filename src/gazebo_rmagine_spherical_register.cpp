#include <gazebo/gazebo.hh>
#include <gazebo_rmagine_plugin/gazebo_rmagine_spherical_plugin.h>



namespace gazebo
{
  class RegisterRmagineSphericalPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterRmagineSphericalPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
        sensors::RegisterRmagineSpherical();
        printf("Loaded my sensor!\n");
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterRmagineSphericalPlugin)
}