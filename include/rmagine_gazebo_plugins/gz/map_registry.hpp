#ifndef RMAGINE_GAZEBO_PLUGINS_GZ_MAP_REGISTRY_HPP
#define RMAGINE_GAZEBO_PLUGINS_GZ_MAP_REGISTRY_HPP

#include <memory>
#include <mutex>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <gz/sim/Entity.hh>

#include <rmagine/map/EmbreeMap.hpp>

namespace rmagine_gazebo_plugins
{

// Maps an Embree geometry id (as seen in raycast results, e.g.
// SimulationResults::object_ids) back to the gz-sim Entity it came from.
// Consumers that need entity-specific data (e.g. radar-specific material
// tags on a <visual>) can resolve it from here instead of re-walking the
// scene the map system already walked once.
using ObjectEntityMap = std::unordered_map<unsigned int, gz::sim::Entity>;

class MapRegistry
{
public:
  static MapRegistry &Instance()
  {
    static MapRegistry instance;
    return instance;
  }

  void SetEmbreeMap(const std::string &key, const rmagine::EmbreeMapPtr &map)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    embree_maps_[key] = map;
    embree_revisions_[key]++;
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

  rmagine::EmbreeMapPtr GetEmbreeMap(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = embree_maps_.find(key);
    if(it != embree_maps_.end())
    {
      return it->second;
    }
    return nullptr;
  }

  uint64_t GetEmbreeMapRevision(const std::string &key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = embree_revisions_.find(key);
    if(it != embree_revisions_.end())
    {
      return it->second;
    }
    return 0;
  }

private:
  MapRegistry() = default;
  MapRegistry(const MapRegistry&) = delete;
  MapRegistry& operator=(const MapRegistry&) = delete;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, rmagine::EmbreeMapPtr> embree_maps_;
  std::unordered_map<std::string, uint64_t> embree_revisions_;
  std::unordered_map<std::string, std::shared_ptr<const ObjectEntityMap>> object_entities_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_GZ_MAP_REGISTRY_HPP
