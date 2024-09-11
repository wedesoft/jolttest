#include <iostream>
#include <cstdarg>
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

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

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;
  return 0;
}
