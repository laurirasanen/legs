#pragma once

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <legs/iphysics.hpp>
#include <legs/log.hpp>

#include "job_system_thread_pool.hpp"

namespace legs
{
class Physics final : public IPhysics
{
  public:
    static void Register();

    Physics();
    ~Physics();

    Physics(const Physics&)            = delete;
    Physics(Physics&&)                 = delete;
    Physics& operator=(const Physics&) = delete;
    Physics& operator=(Physics&&)      = delete;

    void Optimize() override;
    void Update() override;

    JPH::BodyID CreateBody(JPH::BodyCreationSettings settings) override;
    void        AddBody(JPH::BodyID id) override;
    void        RemoveBody(JPH::BodyID id) override;
    void        DestroyBody(JPH::BodyID id) override;

    void GetBodyTransform(JPH::BodyID id, std::shared_ptr<STransform> trans) override;
    void SetBodyTransform(JPH::BodyID id, std::shared_ptr<STransform> trans) override;

    void SetBodyPosition(JPH::BodyID id, glm::vec3 pos) override;
    void SetBodyRotation(JPH::BodyID id, glm::quat rot) override;
    void SetBodyVelocity(JPH::BodyID id, glm::vec3 vel) override;
    void SetBodyAngularVelocity(JPH::BodyID id, glm::vec3 vel) override;

  private:
    JPH::PhysicsSystem     m_physicsSystem;
    JPH::TempAllocatorImpl m_tempAllocator;
    JobSystemThreadPool    m_jobSystem;
    float                  m_maxDeltaTime;
};
}; // namespace legs
