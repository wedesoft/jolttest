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
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>


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

  TempAllocatorMalloc temp_allocator;
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
  physics_system.SetGravity(Vec3(0, -0.4, 0));
  BodyInterface &body_interface = physics_system.GetBodyInterface();

  BoxShapeSettings ground_shape_settings(Vec3(3.0, 0.1, 3.0));
  ground_shape_settings.mConvexRadius = 0.001;
  ground_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult ground_shape_result = ground_shape_settings.Create();
  ShapeRefC ground_shape = ground_shape_result.Get();
  BodyCreationSettings ground_settings(ground_shape, RVec3(0.0, -0.5, 0.0), Quat::sIdentity(), EMotionType::Static, Layers::MOVING);
  Body *ground = body_interface.CreateBody(ground_settings);
  ground->SetFriction(0.5);
  body_interface.AddBody(ground->GetID(), EActivation::DontActivate);

	const float wheel_radius = 0.03f;
	const float wheel_width = 0.02f;
	const float half_vehicle_length = 0.15f;
	const float half_vehicle_width = 0.1f;
	const float half_vehicle_height = 0.02f;
	const float max_steering_angle = DegreesToRadians(30.0f);

	RefConst<Shape> car_shape = new BoxShape(Vec3(half_vehicle_length, half_vehicle_height, half_vehicle_width));
	BodyCreationSettings car_body_settings(car_shape, RVec3::sZero(), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
	car_body_settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
	car_body_settings.mMassPropertiesOverride.mMass = 10.0f;

	VehicleConstraintSettings vehicle;

	WheelSettingsWV *w1 = new WheelSettingsWV;
	w1->mPosition = Vec3(half_vehicle_length - 2.0f * wheel_radius, -0.9f * half_vehicle_height, 0.0f);
	w1->mMaxSteerAngle = max_steering_angle;
	w1->mMaxHandBrakeTorque = 0.0f;
  w1->mRadius = wheel_radius;
  w1->mWidth = wheel_width;

	WheelSettingsWV *w2 = new WheelSettingsWV;
	w2->mPosition = Vec3(-half_vehicle_length + 2.0f * wheel_radius, -0.9f * half_vehicle_height, half_vehicle_width);
	w2->mMaxSteerAngle = 0.0f;
  w2->mRadius = wheel_radius;
  w2->mWidth = wheel_width;

	WheelSettingsWV *w3 = new WheelSettingsWV;
	w3->mPosition = Vec3(-half_vehicle_length + 2.0f * wheel_radius, -0.9f * half_vehicle_height, -half_vehicle_width);
	w3->mMaxSteerAngle = 0.0f;
  w3->mRadius = wheel_radius;
  w3->mWidth = wheel_width;

  vehicle.mWheels = {w1, w2, w3};

	WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
	vehicle.mController = controller;

  Body *car_body = body_interface.CreateBody(car_body_settings);
  body_interface.AddBody(car_body->GetID(), EActivation::Activate);
  VehicleConstraint *c = new VehicleConstraint(*car_body, vehicle);
  VehicleCollisionTester *tester = new VehicleCollisionTesterRay(Layers::MOVING);
  c->SetVehicleCollisionTester(tester);
  physics_system.AddConstraint(c);
  physics_system.AddStepListener(c);

  WheeledVehicleController *vehicle_controller = static_cast<WheeledVehicleController *>(c->GetController());
  vehicle_controller->SetDriverInput(0.0f, 0.0f, 0.0f, 0.0f);

  double t = glfwGetTime();
  while (true) {
    double dt = glfwGetTime() - t;
    body_interface.ActivateBody(c->GetVehicleBody()->GetID());
    RMat44 transform = body_interface.GetWorldTransform(car_body->GetID());
    RVec3 position = transform.GetTranslation();
    cout << position.GetX() << " " << position.GetY() << " " << position.GetZ() << endl;
    const int cCollisionSteps = 1;
    physics_system.Update(dt, cCollisionSteps, &temp_allocator, &job_system);
    t += dt;
  }

  // destruct
  body_interface.RemoveBody(car_body->GetID());
  body_interface.RemoveBody(ground->GetID());

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;

  return 0;
}
