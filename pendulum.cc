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
#include <Jolt/Physics/Constraints/HingeConstraint.h>
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
      return false;
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
    return false;
  }
};

int width = 1280;
int height = 720;

const char *vertexSource = "#version 410 core\n\
uniform float aspect;\n\
uniform vec3 axes;\n\
uniform vec3 translation;\n\
uniform mat3 rotation;\n\
in vec3 point;\n\
in vec3 normal;\n\
out vec3 n;\n\
void main()\n\
{\n\
  n = rotation * normal;\n\
  gl_Position = vec4((rotation * (point * axes) + translation) * vec3(1, aspect, 1), 1);\n\
}";

const char *fragmentSource = "#version 410 core\n\
uniform vec3 light;\n\
in vec3 n;\n\
out vec3 fragColor;\n\
void main()\n\
{\n\
  float ambient = 0.3;\n\
  float diffuse = 0.7 * max(dot(light, n), 0);\n\
  fragColor = vec3(1, 1, 1) * (ambient + diffuse);\n\
}";

// Vertex array data
GLfloat vertices[] = {
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

unsigned int indices[] = {
   0,  1,  2,  3,
   4,  5,  6,  7,
   8,  9, 10, 11,
  12, 13, 14, 15,
  16, 17, 18, 19,
  20, 21, 22, 23
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
  GLFWwindow *window = glfwCreateWindow(width, height, "Double pendulum with Project Chrono", NULL, NULL);
  glfwMakeContextCurrent(window);
  glewInit();

  glClearColor(0.1f, 0.1f, 0.1f, 0.0f);
  glViewport(0, 0, width, height);

  GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertexShader, 1, &vertexSource, NULL);
  glCompileShader(vertexShader);
  handleCompileError("Vertex shader", vertexShader);

  GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
  glCompileShader(fragmentShader);
  handleCompileError("Fragment shader", fragmentShader);

  GLuint program = glCreateProgram();
  glAttachShader(program, vertexShader);
  glAttachShader(program, fragmentShader);
  glLinkProgram(program);
  handleLinkError("Shader program", program);

  GLuint vao;
  GLuint vbo;
  GLuint idx;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glGenBuffers(1, &idx);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, idx);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

  glUseProgram(program);

  glVertexAttribPointer(glGetAttribLocation(program, "point"),
                        3, GL_FLOAT, GL_FALSE,
                        6 * sizeof(float), (void *)0);
  glVertexAttribPointer(glGetAttribLocation(program, "normal"),
                        3, GL_FLOAT, GL_FALSE,
                        6 * sizeof(float), (void *)(3 * sizeof(float)));

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);

  float light[3] = {0.36f, 0.8f, -0.48f};
  glUniform3fv(glGetUniformLocation(program, "light"), 1, light);
  glUniform1f(glGetUniformLocation(program, "aspect"), (float)width / (float)height);
  float a = 0.5;
  float b = 0.05;
  float c = 0.05;
  float axes[3] = {(float)a, (float)b, (float)c};
  glUniform3fv(glGetUniformLocation(program, "axes"), 1, axes);

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

  BoxShapeSettings base_shape_settings(Vec3(0.1, 0.1, 0.1));
  base_shape_settings.mConvexRadius = 0.01;
  base_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult base_shape_result = base_shape_settings.Create();
  ShapeRefC base_shape = base_shape_result.Get();
  BodyCreationSettings base_settings(base_shape, RVec3(0.0, 0.5, 0.0), Quat::sIdentity(), EMotionType::Static, Layers::MOVING);
  Body *base = body_interface.CreateBody(base_settings);
  body_interface.AddBody(base->GetID(), EActivation::DontActivate);

  vector<Body *> pendulum;

  BoxShapeSettings upper_shape_settings(Vec3(a, b, c));
  upper_shape_settings.mConvexRadius = 0.01;
  upper_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult upper_shape_result = upper_shape_settings.Create();
  ShapeRefC upper_shape = upper_shape_result.Get();
  BodyCreationSettings upper_settings(upper_shape, RVec3(0.5 * a, 0.5, 0.0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
  upper_settings.mApplyGyroscopicForce = true;
  upper_settings.mLinearDamping = 0.0;
  upper_settings.mAngularDamping = 0.0;
  Body *upper = body_interface.CreateBody(upper_settings);
  body_interface.AddBody(upper->GetID(), EActivation::Activate);
  pendulum.push_back(upper);

  BoxShapeSettings lower_shape_settings(Vec3(a, b, c));
  lower_shape_settings.mConvexRadius = 0.01;
  lower_shape_settings.SetEmbedded();
  ShapeSettings::ShapeResult lower_shape_result = lower_shape_settings.Create();
  ShapeRefC lower_shape = lower_shape_result.Get();
  BodyCreationSettings lower_settings(lower_shape, RVec3(1.5 * a, 0.5, 0.0), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
  lower_settings.mApplyGyroscopicForce = true;
  lower_settings.mLinearDamping = 0.0;
  lower_settings.mAngularDamping = 0.0;
  Body *lower = body_interface.CreateBody(lower_settings);
  body_interface.AddBody(lower->GetID(), EActivation::Activate);
  pendulum.push_back(lower);

  HingeConstraintSettings hinge1;
  hinge1.mPoint1 = hinge1.mPoint2 = RVec3(0.0, 0.5, 0);
  hinge1.mHingeAxis1 = hinge1.mHingeAxis2 = Vec3::sAxisZ();
  hinge1.mNormalAxis1 = hinge1.mNormalAxis2 = Vec3::sAxisY();
  physics_system.AddConstraint(hinge1.Create(*base, *upper));

  HingeConstraintSettings hinge2;
  hinge2.mPoint1 = hinge2.mPoint2 = RVec3(a, 0.5, 0);
  hinge2.mHingeAxis1 = hinge2.mHingeAxis2 = Vec3::sAxisZ();
  hinge2.mNormalAxis1 = hinge2.mNormalAxis2 = Vec3::sAxisY();
  physics_system.AddConstraint(hinge2.Create(*upper, *lower));

  physics_system.OptimizeBroadPhase();

  double t = glfwGetTime();
  while (!glfwWindowShouldClose(window)) {
    double dt = glfwGetTime() - t;

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    for (auto body=pendulum.begin(); body!=pendulum.end(); body++) {
      RMat44 transform = body_interface.GetWorldTransform((*body)->GetID());
      RVec3 position = transform.GetTranslation();
      Vec3 x = transform.GetAxisX();
      Vec3 y = transform.GetAxisY();
      Vec3 z = transform.GetAxisZ();
      float translation[3] = {(float)position.GetX(), (float)position.GetY(), (float)position.GetZ()};
      glUniform3fv(glGetUniformLocation(program, "translation"), 1, translation);
      float rotation[9] = {x.GetX(), y.GetX(), z.GetX(), x.GetY(), y.GetY(), z.GetY(), x.GetZ(), y.GetZ(), z.GetZ()};
      glUniformMatrix3fv(glGetUniformLocation(program, "rotation"), 1, GL_TRUE, rotation);
      glDrawElements(GL_QUADS, 24, GL_UNSIGNED_INT, (void *)0);
    };

    glfwSwapBuffers(window);
    glfwPollEvents();
    const int cCollisionSteps = 1;
    body_interface.ActivateBody(upper->GetID());
    physics_system.Update(dt, cCollisionSteps, &temp_allocator, &job_system);
    t += dt;
  };

  body_interface.RemoveBody(upper->GetID());
  body_interface.RemoveBody(lower->GetID());
  body_interface.RemoveBody(base->GetID());
  body_interface.DestroyBody(upper->GetID());
  body_interface.DestroyBody(lower->GetID());
  body_interface.DestroyBody(base->GetID());

  UnregisterTypes();
  delete Factory::sInstance;
  Factory::sInstance = nullptr;

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glDeleteBuffers(1, &idx);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDeleteBuffers(1, &vbo);
  glBindVertexArray(0);
  glDeleteVertexArrays(1, &vao);

  glDeleteProgram(program);
  glDeleteShader(vertexShader);
  glDeleteShader(fragmentShader);

  glfwTerminate();
  return 0;
}