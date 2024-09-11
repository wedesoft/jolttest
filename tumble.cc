#include <iostream>
#include <cstdarg>
#include <thread>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>

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

int main(void)
{
  RegisterDefaultAllocator();
  Trace = TraceImpl;
  Factory::sInstance = new Factory();
  RegisterTypes();

  TempAllocatorMalloc allocator;
  JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;
  return 0;
}
