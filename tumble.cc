#include <iostream>
#include <cstdarg>
#include <thread>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>


using namespace std;
using namespace JPH;

static void TraceImpl(const char *inFMT, ...)
{
  va_list list;
  va_start(list, inFMT);
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), inFMT, list);
  va_end(list);
  cerr << buffer << endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{
  cerr << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << endl;
  return true;
};

#endif


namespace Layers
{
  static constexpr ObjectLayer MOVING = 0;
};

class ObjectLayerPairFilterImpl: public ObjectLayerPairFilter
{
  public:
    virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override {
      return true;
    }
};

namespace BroadPhaseLayers
{
  static constexpr BroadPhaseLayer MOVING(0);
};

class BPLayerInterfaceImpl final: public BroadPhaseLayerInterface
{
  public:
    virtual uint GetNumBroadPhaseLayers() const override {
      return 1;
    }

    virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override {
      return BroadPhaseLayers::MOVING;
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char *GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override {
      return "MOVING";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
  virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override {
    return true;
  }
};

int main(void)
{
  RegisterDefaultAllocator();
  Trace = TraceImpl;
  JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
  Factory::sInstance = new Factory();
  RegisterTypes();

  TempAllocatorMalloc allocator;
  JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

  const uint cMaxBodies = 1024;
  const uint cNumBodyMutexes = 0;
  const uint cMaxBodyPairs = 1024;
  const uint cMaxContactConstraints = 1024;
  BPLayerInterfaceImpl broad_phase_layer_interface;
  ObjectLayerPairFilterImpl object_vs_object_layer_filter;
  ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;

  PhysicsSystem physics_system;
  physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface,
                      object_vs_broadphase_layer_filter, object_vs_object_layer_filter);

  BodyInterface &body_interface = physics_system.GetBodyInterface();
  float a = 1.0;
  float b = 0.1;
  float c = 0.5;
  BoxShapeSettings body_shape_settings(Vec3(a, b, c));
  body_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult body_shape_result = body_shape_settings.Create();
  ShapeRefC body_shape = body_shape_result.Get();
  BodyCreationSettings body_settings(body_shape, RVec3(0.0, 0.0, 0.0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
  Body *body = body_interface.CreateBody(body_settings);
  body_interface.AddBody(body->GetID(), EActivation::Activate);

  body_interface.RemoveBody(body->GetID());
	body_interface.DestroyBody(body->GetID());

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;
  return 0;
}
