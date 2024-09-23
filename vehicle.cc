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

int width = 1280;
int height = 720;

const char *vertex_body = "#version 410 core\n\
uniform float aspect;\n\
uniform vec3 axes;\n\
uniform vec3 translation;\n\
uniform mat3 rotation;\n\
in vec3 point;\n\
in vec3 normal;\n\
out vec3 n;\n\
void main()\n\
{\n\
  n = (rotation * normal).zyx;\n\
  gl_Position = vec4(((rotation * (point * axes) + translation) * vec3(1, aspect, 1)).zyx, 1);\n\
}";

const char *fragment_body = "#version 410 core\n\
uniform vec3 light;\n\
in vec3 n;\n\
out vec3 fragColor;\n\
void main()\n\
{\n\
  float ambient = 0.3;\n\
  float diffuse = 0.7 * max(dot(light, n), 0);\n\
  fragColor = vec3(1, 1, 1) * (ambient + diffuse);\n\
}";

const char *vertex_wheel = "#version 410 core\n\
uniform float aspect;\n\
uniform float radius;\n\
uniform int num_points;\n\
uniform vec3 translation;\n\
uniform mat3 rotation;\n\
in vec3 point;\n\
out vec3 color;\n\
void main()\n\
{\n\
  vec3 radius_vector = radius * vec3(0, sin(2.0 * 3.1415926 * gl_InstanceID / num_points), cos(2.0 * 3.1415926 * gl_InstanceID / num_points));\n\
  if (gl_InstanceID == 0)\n\
    color = vec3(1, 0, 0);\n\
  else\n\
    color = vec3(1, 1, 1);\n\
  gl_Position = vec4(((rotation * (point + radius_vector) + translation) * vec3(1, aspect, 1)).zyx, 1);\n\
}";

const char *fragment_wheel = "#version 410 core\n\
in vec3 color;\n\
out vec3 fragColor;\n\
void main()\n\
{\n\
  fragColor = color;\n\
  fragColor = vec3(1, 1, 1);\n\
}";

// Vertex array data
GLfloat vertices_body[] = {
  // Front face
  -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
   0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
  -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

  // Back face
  -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
   0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
   0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
  -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

  // Left face
  -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
  -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,

  // Right face
   0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
   0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
   0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,

  // Top face
  -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
   0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
  -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,

  // Bottom face
  -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
   0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
   0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
  -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f
};

unsigned int indices_body[] = {
   0,  1,  2,  3,
   4,  5,  6,  7,
   8,  9, 10, 11,
  12, 13, 14, 15,
  16, 17, 18, 19,
  20, 21, 22, 23
};

GLfloat vertices_wheel[] = {
   0.0f,  0.0f,  0.0f
};

unsigned int indices_wheel[] = {
   0
};

void handleCompileError(const char *step, GLuint shader)
{
  GLint result = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &result);
  if (result == GL_FALSE) {
    char buffer[1024];
    glGetShaderInfoLog(shader, 1024, NULL, buffer);
    if (buffer[0])
      fprintf(stderr, "%s: %s\n", step, buffer);
  };
}

void handleLinkError(const char *step, GLuint program)
{
  GLint result = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &result);
  if (result == GL_FALSE) {
    char buffer[1024];
    glGetProgramInfoLog(program, 1024, NULL, buffer);
    if (buffer[0])
      fprintf(stderr, "%s: %s\n", step, buffer);
  };
}

int main(void)
{
  glfwInit();
  GLFWwindow *window = glfwCreateWindow(width, height, "Falling stack of boxes with Jolt Physics", NULL, NULL);
  glfwMakeContextCurrent(window);
  glewInit();

  const float wheel_radius = 0.03f;
  const float wheel_width = 0.02f;
  int num_points = 18;
  const float half_vehicle_length = 0.15f;
  const float half_vehicle_width = 0.1f;
  const float half_vehicle_height = 0.02f;
  const float max_steering_angle = DegreesToRadians(30.0f);

  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
  glViewport(0, 0, width, height);

  GLuint vertex_shader_body = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader_body, 1, &vertex_body, NULL);
  glCompileShader(vertex_shader_body);
  handleCompileError("Vertex shader", vertex_shader_body);

  GLuint fragment_shader_body = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader_body, 1, &fragment_body, NULL);
  glCompileShader(fragment_shader_body);
  handleCompileError("Fragment shader", fragment_shader_body);

  GLuint program_body = glCreateProgram();
  glAttachShader(program_body, vertex_shader_body);
  glAttachShader(program_body, fragment_shader_body);
  glLinkProgram(program_body);
  handleLinkError("Shader program", program_body);

  GLuint vao_body;
  GLuint vbo_body;
  GLuint idx_body;

  glGenVertexArrays(1, &vao_body);
  glBindVertexArray(vao_body);

  glGenBuffers(1, &vbo_body);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_body);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_body), vertices_body, GL_STATIC_DRAW);
  glGenBuffers(1, &idx_body);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_body);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_body), indices_body, GL_STATIC_DRAW);

  glUseProgram(program_body);

  glVertexAttribPointer(glGetAttribLocation(program_body, "point"),
                        3, GL_FLOAT, GL_FALSE,
                        6 * sizeof(float), (void *)0);
  glVertexAttribPointer(glGetAttribLocation(program_body, "normal"),
                        3, GL_FLOAT, GL_FALSE,
                        6 * sizeof(float), (void *)(3 * sizeof(float)));

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glPointSize(2.0f);

  GLuint vertex_shader_wheel = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader_wheel, 1, &vertex_wheel, NULL);
  glCompileShader(vertex_shader_wheel);
  handleCompileError("Vertex shader", vertex_shader_wheel);

  GLuint fragment_shader_wheel = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader_wheel, 1, &fragment_wheel, NULL);
  glCompileShader(fragment_shader_wheel);
  handleCompileError("Fragment shader", fragment_shader_wheel);

  GLuint program_wheel = glCreateProgram();
  glAttachShader(program_wheel, vertex_shader_wheel);
  glAttachShader(program_wheel, fragment_shader_wheel);
  glLinkProgram(program_wheel);
  handleLinkError("Shader program", program_wheel);

  float light[3] = {0.36f, 0.8f, -0.48f};
  glUniform3fv(glGetUniformLocation(program_body, "light"), 1, light);
  glUniform1f(glGetUniformLocation(program_body, "aspect"), (float)width / (float)height);
  float a = half_vehicle_width * 2.0f;;
  float b = half_vehicle_height * 2.0f;
  float c = half_vehicle_length * 2.0f;
  float axes[3] = {a, b, c};
  glUniform3fv(glGetUniformLocation(program_body, "axes"), 1, axes);

  GLuint vao_wheel;
  GLuint vbo_wheel;
  GLuint idx_wheel;

  glGenVertexArrays(1, &vao_wheel);
  glBindVertexArray(vao_wheel);

  glGenBuffers(1, &vbo_wheel);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_wheel);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices_wheel), vertices_wheel, GL_STATIC_DRAW);
  glGenBuffers(1, &idx_wheel);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_wheel);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices_wheel), indices_wheel, GL_STATIC_DRAW);

  glUseProgram(program_wheel);

  glVertexAttribPointer(glGetAttribLocation(program_wheel, "point"),
                        3, GL_FLOAT, GL_FALSE,
                        3 * sizeof(float), (void *)0);

  glEnableVertexAttribArray(0);

  glUniform1f(glGetUniformLocation(program_wheel, "aspect"), (float)width / (float)height);
  glUniform1f(glGetUniformLocation(program_wheel, "radius"), wheel_radius);
  glUniform1i(glGetUniformLocation(program_wheel, "num_points"), num_points);

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

  BoxShapeSettings ground_shape_settings(Vec3(2000.0, 0.1, 2000.0));
  ground_shape_settings.mConvexRadius = 0.001;
  ground_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult ground_shape_result = ground_shape_settings.Create();
  ShapeRefC ground_shape = ground_shape_result.Get();
  BodyCreationSettings ground_settings(ground_shape, RVec3(0.0, -0.5, 0.0), Quat::sIdentity(), EMotionType::Static, Layers::MOVING);
  Body *ground = body_interface.CreateBody(ground_settings);
  ground->SetFriction(0.5);
  ground->SetRestitution(0.3f);
  body_interface.AddBody(ground->GetID(), EActivation::DontActivate);

  RefConst<Shape> car_shape = new BoxShape(Vec3(half_vehicle_width, half_vehicle_height, half_vehicle_length));
  BodyCreationSettings car_body_settings(car_shape, RVec3::sZero(), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
  car_body_settings.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
  car_body_settings.mMassPropertiesOverride.mMass = 1500.0f;

  VehicleConstraintSettings vehicle;

  WheelSettingsWV *w1 = new WheelSettingsWV;
  w1->mPosition = Vec3(0.0f, -0.9f * half_vehicle_height, half_vehicle_length - 1.0f * wheel_radius);
  w1->mSuspensionMinLength = wheel_radius;
  w1->mSuspensionMaxLength = 2 * wheel_radius;
  w1->mMaxSteerAngle = max_steering_angle;
  w1->mMaxHandBrakeTorque = 0.0f;
  w1->mRadius = wheel_radius;
  w1->mWidth = wheel_width;

  WheelSettingsWV *w2 = new WheelSettingsWV;
  w2->mPosition = Vec3(half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 1.0f * wheel_radius);
  w2->mSuspensionMinLength = wheel_radius;
  w2->mSuspensionMaxLength = 2 * wheel_radius;
  w2->mMaxSteerAngle = 0.0f;
  w2->mRadius = wheel_radius;
  w2->mWidth = wheel_width;

  WheelSettingsWV *w3 = new WheelSettingsWV;
  w3->mPosition = Vec3(-half_vehicle_width, -0.9f * half_vehicle_height, -half_vehicle_length + 1.0f * wheel_radius);
  w3->mSuspensionMinLength = wheel_radius;
  w3->mSuspensionMaxLength = 2 * wheel_radius;
  w3->mMaxSteerAngle = 0.0f;
  w3->mRadius = wheel_radius;
  w3->mWidth = wheel_width;

  vehicle.mWheels = {w1, w2, w3};

  WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
  vehicle.mController = controller;

  Body *car_body = body_interface.CreateBody(car_body_settings);
  body_interface.AddBody(car_body->GetID(), EActivation::Activate);
  VehicleConstraint *constraint = new VehicleConstraint(*car_body, vehicle);
  VehicleCollisionTester *tester = new VehicleCollisionTesterRay(Layers::MOVING);
  constraint->SetVehicleCollisionTester(tester);
  physics_system.AddConstraint(constraint);
  physics_system.AddStepListener(constraint);

  WheeledVehicleController *vehicle_controller = static_cast<WheeledVehicleController *>(constraint->GetController());
  vehicle_controller->SetDriverInput(0.0f, 0.0f, 0.0f, 0.0f);

  body_interface.SetLinearVelocity(car_body->GetID(), Vec3(0.0f, 0.0f, 2.0f));

  double t = glfwGetTime();
  while (!glfwWindowShouldClose(window)) {
    double dt = glfwGetTime() - t;
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    body_interface.ActivateBody(constraint->GetVehicleBody()->GetID());

    glUseProgram(program_body);
    glBindVertexArray(vao_body);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_body);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_body);
    RMat44 transform = body_interface.GetWorldTransform(car_body->GetID());
    RVec3 position = transform.GetTranslation();
    double pz = position.GetZ();
    while (pz >= 1.0)
      pz -= 2.0;
    double dz = pz - position.GetZ();
    Vec3 x = transform.GetAxisX();
    Vec3 y = transform.GetAxisY();
    Vec3 z = transform.GetAxisZ();
    float translation[3] = {(float)position.GetX(), (float)position.GetY(), (float)(position.GetZ() + dz)};
    glUniform3fv(glGetUniformLocation(program_body, "translation"), 1, translation);
    float rotation[9] = {x.GetX(), y.GetX(), z.GetX(), x.GetY(), y.GetY(), z.GetY(), x.GetZ(), y.GetZ(), z.GetZ()};
    glUniformMatrix3fv(glGetUniformLocation(program_body, "rotation"), 1, GL_TRUE, rotation);
    glDrawElements(GL_QUADS, 24, GL_UNSIGNED_INT, (void *)0);

    glUseProgram(program_wheel);
    glBindVertexArray(vao_wheel);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_wheel);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx_wheel);
    for (int i=0; i<3; i++) {
      RMat44 transform = constraint->GetWheelWorldTransform(i, Vec3::sAxisX(), Vec3::sAxisZ());
      RVec3 position = transform.GetTranslation();
      Vec3 x = transform.GetAxisX();
      Vec3 y = transform.GetAxisY();
      Vec3 z = transform.GetAxisZ();
      float translation[3] = {(float)position.GetX(), (float)position.GetY(), (float)(position.GetZ() + dz)};
      glUniform3fv(glGetUniformLocation(program_wheel, "translation"), 1, translation);
      float rotation[9] = {x.GetX(), y.GetX(), z.GetX(), x.GetY(), y.GetY(), z.GetY(), x.GetZ(), y.GetZ(), z.GetZ()};
      glUniformMatrix3fv(glGetUniformLocation(program_wheel, "rotation"), 1, GL_TRUE, rotation);
      glDrawElementsInstanced(GL_POINTS, 1, GL_UNSIGNED_INT, (void *)0, num_points);
    };

    glfwSwapBuffers(window);
    glfwPollEvents();
    const int cCollisionSteps = 1;
    physics_system.Update(dt, cCollisionSteps, &temp_allocator, &job_system);
    t += dt;
  }

  physics_system.RemoveStepListener(constraint);

  // destruct
  body_interface.RemoveBody(car_body->GetID());
  body_interface.RemoveBody(ground->GetID());

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  glDeleteBuffers(1, &idx_body);
  glDeleteBuffers(1, &vbo_body);
  glDeleteVertexArrays(1, &vao_body);

  glDeleteBuffers(1, &idx_wheel);
  glDeleteBuffers(1, &vbo_wheel);
  glDeleteVertexArrays(1, &vao_wheel);

  glDeleteProgram(program_body);
  glDeleteShader(vertex_shader_body);
  glDeleteShader(fragment_shader_body);

  glfwTerminate();
  return 0;
}
