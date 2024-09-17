#pragma once

#include <memory>
#include <string>

#include <legs/iphysics.hpp>

#include <legs/entity/mesh_entity.hpp>
#include <legs/entry.hpp>

namespace legs
{
class PhysicsEntity : public MeshEntity
{
  public:
    PhysicsEntity() : MeshEntity()
    {
    }

    virtual ~PhysicsEntity() = default;

    PhysicsEntity(const PhysicsEntity&)            = delete;
    PhysicsEntity(PhysicsEntity&&)                 = delete;
    PhysicsEntity& operator=(const PhysicsEntity&) = delete;
    PhysicsEntity& operator=(PhysicsEntity&&)      = delete;

    virtual void OnSpawn() override
    {
        MeshEntity::OnSpawn();
        m_joltBody = g_engine->GetWorld()->GetPhysics()->CreateBody({});
        g_engine->GetWorld()->GetPhysics()->AddBody(m_joltBody);
    }

    virtual void OnDestroy() override
    {
        MeshEntity::OnDestroy();
        g_engine->GetWorld()->GetPhysics()->RemoveBody(m_joltBody);
        g_engine->GetWorld()->GetPhysics()->DestroyBody(m_joltBody);
    }

    virtual void OnFrame() override
    {
        MeshEntity::OnFrame();
    }

    virtual void OnTick() override
    {
        MeshEntity::OnTick();
        auto trans          = g_engine->GetWorld()->GetPhysics()->GetBodyTransform(m_joltBody);
        Transform->position = trans.position;
        Transform->rotation = trans.rotation;
    }

  protected:
    JPH::BodyID m_joltBody;
};
}; // namespace legs
