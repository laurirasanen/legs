#pragma once

#include <memory>
#include <string>

#include <legs/components/transform.hpp>

namespace legs
{
class Entity
{
  public:
    Entity() : Name(""), Transform(std::make_shared<STransform>()) {};
    virtual ~Entity() = default;

    Entity(const Entity&)            = delete;
    Entity(Entity&&)                 = delete;
    Entity& operator=(const Entity&) = delete;
    Entity& operator=(Entity&&)      = delete;

    virtual void OnSpawn() {};
    virtual void OnDestroy() {};
    virtual void OnFrame() {};
    virtual void OnTick() {};

    std::string                 Name;
    std::shared_ptr<STransform> Transform;
};
}; // namespace legs
