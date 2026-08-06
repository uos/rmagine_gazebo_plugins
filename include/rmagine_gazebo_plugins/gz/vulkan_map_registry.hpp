#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_VULKAN_MAP_REGISTRY_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_VULKAN_MAP_REGISTRY_HPP

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <gz/sim/Entity.hh>

#include <rmagine/map/VulkanMap.hpp>

namespace rmagine_gazebo_plugins
{

// GPU counterpart of MapRegistry (map_registry.hpp), for the Vulkan backend.
// Kept as its own singleton/header rather than added to MapRegistry or
// OptixMapRegistry so that packages/targets which only need the CPU (Embree)
// or CUDA (OptiX) path never have to see VulkanMap.hpp or link Vulkan --
// same rationale as optix_map_registry.hpp's separation from MapRegistry.
//
// ObjectEntityMap is redeclared here (not #include-d from map_registry.hpp
// or optix_map_registry.hpp) deliberately: those headers pull in
// EmbreeMap.hpp/OptixMap.hpp respectively, which would make the Vulkan-only
// .so's (rmagine_vulkan_map_system/rmagine_vulkan_sensor_system) depend on
// Embree/OptiX headers too, even when only rmagine::vulkan is built.
using ObjectEntityMap = std::unordered_map<unsigned int, gz::sim::Entity>;

class VulkanMapRegistry
{
public:
  static VulkanMapRegistry &Instance()
  {
    static VulkanMapRegistry instance;
    return instance;
  }

  void SetVulkanMap(const std::string &key, const rmagine::VulkanMapPtr &map)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vulkan_maps_[key] = map;
    vulkan_revisions_[key]++;
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

  rmagine::VulkanMapPtr GetVulkanMap(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = vulkan_maps_.find(key);
    if(it != vulkan_maps_.end())
    {
      return it->second;
    }
    return nullptr;
  }

  uint64_t GetVulkanMapRevision(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = vulkan_revisions_.find(key);
    if(it != vulkan_revisions_.end())
    {
      return it->second;
    }
    return 0;
  }

  // See the identical comment on MapRegistry::SetMapMutex/GetMapMutex
  // (map_registry.hpp) -- same rationale, Vulkan side.
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
  VulkanMapRegistry() = default;
  VulkanMapRegistry(const VulkanMapRegistry&) = delete;
  VulkanMapRegistry& operator=(const VulkanMapRegistry&) = delete;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, rmagine::VulkanMapPtr> vulkan_maps_;
  std::unordered_map<std::string, uint64_t> vulkan_revisions_;
  std::unordered_map<std::string, std::shared_ptr<const ObjectEntityMap>> object_entities_;
  std::unordered_map<std::string, std::shared_ptr<std::shared_mutex>> map_mutexes_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_VULKAN_MAP_REGISTRY_HPP
