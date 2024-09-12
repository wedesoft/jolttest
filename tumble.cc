#include <iostream>
#include <cstdarg>
#include <thread>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
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

class ObjectLayerPairFilterImpl: public ObjectLayerPairFilter
{
  public:
    virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override {
      return true;
    }
};

int main(void)
{
  RegisterDefaultAllocator();
  Trace = TraceImpl;
  Factory::sInstance = new Factory();
  RegisterTypes();

  TempAllocatorMalloc allocator;
  JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

  // const uint cMaxBodies = 1024;
  // const uint cNumBodyMutexes = 0;
  // const uint cMaxBodyPairs = 1024;
  // const uint cMaxContactConstraints = 1024;

  ObjectLayerPairFilterImpl object_vs_object_layer_filter;

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;
  return 0;
}
