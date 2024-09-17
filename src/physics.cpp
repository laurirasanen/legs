#include <cmath>
#include <cstdarg>
#include <iostream>
#include <thread>

#include <legs/time.hpp>

#include "physics.hpp"

namespace legs
{

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char* fmt, ...)
{
    va_list list;
    va_start(list, fmt);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), fmt, list);
    va_end(list);

    LOG_ERROR("JOLT TRACE: {}", buffer);
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(
    const char* inExpression,
    const char* inMessage,
    const char* inFile,
    uint        inLine
)
{
    Log::Print(inFile, inLine, inExpression, LogLevel::Error, inMessage);

    // Breakpoint
    return true;
};

#endif // JPH_ENABLE_ASSERTS

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but
// you can have more layers if you want. E.g. you could have a layer for high detail collision
// (which is not used by the physics simulation but only if you do collision testing).
namespace Layers
{
static constexpr JPH::ObjectLayer NON_MOVING = 0;
static constexpr JPH::ObjectLayer MOVING     = 1;
static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}; // namespace Layers

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
  public:
    virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2)
        const override
    {
        switch (inObject1)
        {
            case Layers::NON_MOVING:
                return inObject2 == Layers::MOVING; // Non moving only collides with moving
            case Layers::MOVING:
                return true; // Moving collides with everything
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least
// want to have a layer for non-moving and moving objects to avoid having to update a tree full of
// static objects every frame. You can have a 1-on-1 mapping between object layers and broadphase
// layers (like in this case) but if you have many object layers you'll be creating many broad phase
// trees, which is not efficient. If you want to fine tune your broadphase layers define
// JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
static constexpr JPH::BroadPhaseLayer MOVING(1);
static constexpr uint                 NUM_LAYERS(2);
}; // namespace BroadPhaseLayers

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
  public:
    BPLayerInterfaceImpl()
    {
        // Create a mapping table from object to broad phase layer
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING]     = BroadPhaseLayers::MOVING;
    }

    virtual uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
        return mObjectToBroadPhase[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((JPH::BroadPhaseLayer::Type)inLayer)
        {
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:
                return "NON_MOVING";
            case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:
                return "MOVING";
            default:
                JPH_ASSERT(false);
                return "INVALID";
        }
    }
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

  private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
  public:
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2)
        const override
    {
        switch (inLayer1)
        {
            case Layers::NON_MOVING:
                return inLayer2 == BroadPhaseLayers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                JPH_ASSERT(false);
                return false;
        }
    }
};

// An example contact listener
class MyContactListener : public JPH::ContactListener
{
  public:
    // See: ContactListener
    virtual JPH::ValidateResult OnContactValidate(
        const JPH::Body&               inBody1,
        const JPH::Body&               inBody2,
        JPH::RVec3Arg                  inBaseOffset,
        const JPH::CollideShapeResult& inCollisionResult
    ) override
    {
        std::cout << "Contact validate callback" << std::endl;

        // Allows you to ignore a contact before it is created (using layers to not make objects
        // collide is cheaper!)
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(
        const JPH::Body&            inBody1,
        const JPH::Body&            inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings&       ioSettings
    ) override
    {
        std::cout << "A contact was added" << std::endl;
    }

    virtual void OnContactPersisted(
        const JPH::Body&            inBody1,
        const JPH::Body&            inBody2,
        const JPH::ContactManifold& inManifold,
        JPH::ContactSettings&       ioSettings
    ) override
    {
        std::cout << "A contact was persisted" << std::endl;
    }

    virtual void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override
    {
        std::cout << "A contact was removed" << std::endl;
    }
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener
{
  public:
    virtual void OnBodyActivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override
    {
        std::cout << "A body got activated" << std::endl;
    }

    virtual void OnBodyDeactivated(const JPH::BodyID& inBodyID, uint64_t inBodyUserData) override
    {
        std::cout << "A body went to sleep" << std::endl;
    }
};

void Physics::Register()
{
    // Register allocation hook. In this example we'll just let Jolt use malloc / free but you can
    // override these if you want (see Memory.h). This needs to be done before any other Jolt
    // function is called.
    JPH::RegisterDefaultAllocator();

    // Install trace and assert callbacks
    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    // Create a factory, this class is responsible for creating instances of classes based on their
    // name or hash and is mainly used for deserialization of saved data. It is not directly used in
    // this example but still required.
    JPH::Factory::sInstance = new JPH::Factory();

    // Register all physics types with the factory and install their collision handlers with the
    // CollisionDispatch class. If you have your own custom shape types you probably need to
    // register their handlers with the CollisionDispatch before calling this function. If you
    // implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it
    // before this function or else this function will create one for you.
    JPH::RegisterTypes();
}

Physics::Physics() :
    m_tempAllocator(10 * 1024 * 1024),
    m_jobSystem(
        JPH::cMaxPhysicsJobs,
        JPH::cMaxPhysicsBarriers,
        static_cast<int>(std::thread::hardware_concurrency()) - 1
    ),
    m_maxDeltaTime(1.0f / 60.0f)
{
    // This is the max amount of rigid bodies that you can add to the physics system. If you try to
    // add more you'll get an error. Note: This value is low because this is a simple test. For a
    // real project use something in the order of 65536.
    const uint cMaxBodies = 1024;

    // This determines how many mutexes to allocate to protect rigid bodies from concurrent access.
    // Set it to 0 for the default settings.
    const uint cNumBodyMutexes = 0;

    // This is the max amount of body pairs that can be queued at any time (the broad phase will
    // detect overlapping body pairs based on their bounding boxes and will insert them into a queue
    // for the narrowphase). If you make this buffer too small the queue will fill up and the broad
    // phase jobs will start to do narrow phase work. This is slightly less efficient. Note: This
    // value is low because this is a simple test. For a real project use something in the order of
    // 65536.
    const uint cMaxBodyPairs = 1024;

    // This is the maximum size of the contact constraint buffer. If more contacts (collisions
    // between bodies) are detected than this number then these contacts will be ignored and bodies
    // will start interpenetrating / fall through the world. Note: This value is low because this is
    // a simple test. For a real project use something in the order of 10240.
    const uint cMaxContactConstraints = 1024;

    // Create mapping table from object layer to broadphase layer
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
    // needs to stay alive!
    BPLayerInterfaceImpl broadPhaseLayerInterface;

    // Create class that filters object vs broadphase layers
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
    // needs to stay alive!
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadphaseLayerFilter;

    // Create class that filters object vs object layers
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance
    // needs to stay alive!
    ObjectLayerPairFilterImpl objectVsObjectLayerFilter;

    // Now we can create the actual physics system.
    m_physicsSystem.Init(
        cMaxBodies,
        cNumBodyMutexes,
        cMaxBodyPairs,
        cMaxContactConstraints,
        broadPhaseLayerInterface,
        objectVsBroadphaseLayerFilter,
        objectVsObjectLayerFilter
    );

    // A body activation listener gets notified when bodies activate and go to sleep
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    MyBodyActivationListener bodyActivationListener;
    m_physicsSystem.SetBodyActivationListener(&bodyActivationListener);

    // A contact listener gets notified when bodies (are about to) collide, and when they separate
    // again. Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    MyContactListener contactListener;
    m_physicsSystem.SetContactListener(&contactListener);
}

Physics::~Physics()
{
    // Unregisters all types with the factory and cleans up the default material
    JPH::UnregisterTypes();

    // Destroy the factory
    if (JPH::Factory::sInstance != nullptr)
    {
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

void Physics::Optimize()
{
    m_physicsSystem.OptimizeBroadPhase();
}

void Physics::Update()
{
    const unsigned int steps = std::ceil(Time::DeltaTick / m_maxDeltaTime);
    m_physicsSystem.Update(Time::DeltaTick, steps, &m_tempAllocator, &m_jobSystem);
}

JPH::BodyID Physics::CreateBody(JPH::BodyCreationSettings settings)
{
    auto body = m_physicsSystem.GetBodyInterface().CreateBody(settings);
    if (body != nullptr)
    {
        return body->GetID();
    }
    return JPH::BodyID(JPH::BodyID::cInvalidBodyID);
}

void Physics::AddBody(JPH::BodyID id)
{
    m_physicsSystem.GetBodyInterface().AddBody(id, JPH::EActivation::Activate);
}

void Physics::RemoveBody(JPH::BodyID id)
{
    m_physicsSystem.GetBodyInterface().RemoveBody(id);
}

void Physics::DestroyBody(JPH::BodyID id)
{
    m_physicsSystem.GetBodyInterface().DestroyBody(id);
}

STransform Physics::GetBodyTransform(JPH::BodyID id)
{
    STransform trans;
    JPH::RVec3 joltPos;
    JPH::Quat  joltRot;

    m_physicsSystem.GetBodyInterface().GetPositionAndRotation(id, joltPos, joltRot);

    trans.position.x = joltPos.GetX();
    trans.position.y = joltPos.GetY();
    trans.position.z = joltPos.GetZ();

    trans.rotation.quaternion.x = joltRot.GetX();
    trans.rotation.quaternion.y = joltRot.GetY();
    trans.rotation.quaternion.z = joltRot.GetZ();
    trans.rotation.quaternion.w = joltRot.GetW();

    return trans;
}

void Physics::SetBodyTransform(JPH::BodyID id, STransform trans)
{
    JPH::RVec3 joltPos = {trans.position.x, trans.position.y, trans.position.z};
    JPH::Quat  joltRot = {
        trans.rotation.quaternion.x,
        trans.rotation.quaternion.y,
        trans.rotation.quaternion.w,
        trans.rotation.quaternion.z
    };

    m_physicsSystem.GetBodyInterface()
        .SetPositionAndRotation(id, joltPos, joltRot, JPH::EActivation::Activate);
}
}; // namespace legs
