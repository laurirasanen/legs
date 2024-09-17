#pragma once

#include <memory>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <legs/components/transform.hpp>

namespace legs
{
class IPhysics
{
  public:
    IPhysics()          = default;
    virtual ~IPhysics() = default;

    IPhysics(const IPhysics&)            = delete;
    IPhysics(IPhysics&&)                 = delete;
    IPhysics& operator=(const IPhysics&) = delete;
    IPhysics& operator=(IPhysics&&)      = delete;

    virtual void Update()   = 0;
    virtual void Optimize() = 0;

    virtual JPH::BodyID CreateBody(JPH::BodyCreationSettings settings) = 0;
    virtual void        AddBody(JPH::BodyID id)                        = 0;
    virtual void        RemoveBody(JPH::BodyID id)                     = 0;
    virtual void        DestroyBody(JPH::BodyID id)                    = 0;

    virtual STransform GetBodyTransform(JPH::BodyID id)                   = 0;
    virtual void       SetBodyTransform(JPH::BodyID id, STransform trans) = 0;
};
}; // namespace legs
