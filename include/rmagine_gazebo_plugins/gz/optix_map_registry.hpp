#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_OPTIX_MAP_REGISTRY_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_OPTIX_MAP_REGISTRY_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <gz/sim/Entity.hh>

#include <rmagine/map/OptixMap.hpp>

namespace rmagine_gazebo_plugins
{

// GPU counterpart of MapRegistry (map_registry.hpp). Kept as its own
// singleton/header rather than added to MapRegistry so that packages/targets
// which only need the CPU (Embree) path never have to see OptixMap.hpp or
// link CUDA/OptiX -- see MIGRATION_HANDOFF.md for why.
//
// ObjectEntityMap is redeclared here (not #include-d from map_registry.hpp)
// deliberately: that header also pulls in EmbreeMap.hpp, which would make
// the OptiX-only .so's (rmagine_optix_map_system/rmagine_optix_sensor_system)
// depend on Embree headers too, even when only rmagine::optix is built.
using ObjectEntityMap = std::unordered_map<unsigned int, gz::sim::Entity>;

class OptixMapRegistry
{
public:
  static OptixMapRegistry &Instance()
  {
    static OptixMapRegistry instance;
    return instance;
  }

  void SetOptixMap(const std::string &key, const rmagine::OptixMapPtr &map)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    optix_maps_[key] = map;
    optix_revisions_[key]++;
  }

  void SetObjectEntities(const std::string &key, const std::shared_ptr<const ObjectEntityMap> &entities)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    object_entities_[key] = entities;
  }

  std::shared_ptr<const ObjectEntityMap> GetObjectEntities(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = object_entities_.find(key);
    if(it != object_entities_.end())
    {
      return it->second;
    }
    return nullptr;
  }

  rmagine::OptixMapPtr GetOptixMap(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = optix_maps_.find(key);
    if(it != optix_maps_.end())
    {
      return it->second;
    }
    return nullptr;
  }

  uint64_t GetOptixMapRevision(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = optix_revisions_.find(key);
    if(it != optix_revisions_.end())
    {
      return it->second;
    }
    return 0;
  }

  // See the identical comment on MapRegistry::SetMapMutex/GetMapMutex
  // (map_registry.hpp) -- same rationale, GPU side.
  void SetMapMutex(const std::string &key, const std::shared_ptr<std::shared_mutex> &map_mutex)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_mutexes_[key] = map_mutex;
  }

  std::shared_ptr<std::shared_mutex> GetMapMutex(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = map_mutexes_.find(key);
    if(it != map_mutexes_.end())
    {
      return it->second;
    }
    return nullptr;
  }

private:
  OptixMapRegistry() = default;
  OptixMapRegistry(const OptixMapRegistry&) = delete;
  OptixMapRegistry& operator=(const OptixMapRegistry&) = delete;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, rmagine::OptixMapPtr> optix_maps_;
  std::unordered_map<std::string, uint64_t> optix_revisions_;
  std::unordered_map<std::string, std::shared_ptr<const ObjectEntityMap>> object_entities_;
  std::unordered_map<std::string, std::shared_ptr<std::shared_mutex>> map_mutexes_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_OPTIX_MAP_REGISTRY_HPP
