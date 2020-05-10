#include "bullet_demo.h"
#include "bullet_demo_shader.cpp"
#include <vector>

enum rigid_body_type
{
	RigidBody_Plane,
	RigidBody_Box,
    RigidBody_Sphere,
};
struct rigid_body
{
    rigid_body_type Type;
    btRigidBody* BulletRigidBody;

    // TODO(georgy): It is not the best place for color
    vec3 Color;
};

class physics
{
    private:
        btDefaultCollisionConfiguration* CollisionConfiguration;
        btDispatcher* Dispatcher;
        btBroadphaseInterface *Broadphase;
        btConstraintSolver* Solver;
        btDiscreteDynamicsWorld* DynamicsWorld;

        u32 MaxRigidBodies;

    public:
        void Initialize(void);
        void Update(r32 dt);

        rigid_body CreateRigidBody(rigid_body_type Type, r32 Mass, btCollisionShape *CollisionShape, r32 Restitution, mat4 &TransformOpenGL, vec3 Color);
        void AddRigidBody(rigid_body RigidBody);

        std::vector<rigid_body> RigidBodies;
};

void
physics::Initialize(void)
{
	CollisionConfiguration = new btDefaultCollisionConfiguration();
	Dispatcher = new btCollisionDispatcher(CollisionConfiguration);
	Broadphase = new btAxisSweep3(btVector3(-500.0f, -500.0f, -500.0f),
                                  btVector3(-500.0f, -500.0f, -500.0f));
	Solver = new btSequentialImpulseConstraintSolver();
	DynamicsWorld = new btDiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConfiguration);

    MaxRigidBodies = 1024;
	
    DynamicsWorld->setGravity(btVector3(0.0f, -9.8f, 0.0f));
}

void
physics::Update(r32 dt)
{
    DynamicsWorld->stepSimulation(dt, 5);
}

rigid_body
physics::CreateRigidBody(rigid_body_type Type, r32 Mass, btCollisionShape *CollisionShape, r32 Restitution, mat4 &TransformOpenGL, vec3 Color)
{
    rigid_body RigidBody = {};

    RigidBody.Type = Type;

    btTransform Transform;
    Transform.setFromOpenGLMatrix(TransformOpenGL.E);
    btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);
    btVector3 LocalInertia(0, 0, 0);
    if(Mass > 0.0f)
    {
        CollisionShape->calculateLocalInertia(Mass, LocalInertia);
    }
    RigidBody.BulletRigidBody = new btRigidBody(Mass, MotionState, CollisionShape, LocalInertia);

    RigidBody.BulletRigidBody->setRestitution(Restitution);

    RigidBody.Color = Color;

    return(RigidBody);
}

void 
physics::AddRigidBody(rigid_body RigidBody)
{
    if(RigidBodies.size() < MaxRigidBodies)
    {
        DynamicsWorld->addRigidBody(RigidBody.BulletRigidBody);
        RigidBodies.push_back(RigidBody);
    }
}

static r32
GetRandomR32Between(r32 A, r32 B)
{
	r32 Result = A + (B - A)*((r32)rand() / (r32)RAND_MAX);

	return(Result);
}

static void
GetSphereVertices(std::vector<vec3> &Vertices, std::vector<u32> &Indices, u32 ParallelCount, u32 MeridianCount)
{
    r32 Radius = 0.5f;

    Vertices.push_back(Radius*vec3(0.0f, 1.0f, 0.0f));
    Vertices.push_back(vec3(0.0f, 1.0f, 0.0f));
    for(u32 Parallel = 0;
        Parallel < ParallelCount;
        Parallel++)
    {
        r32 Phi = -((r32)(Parallel + 1) / (ParallelCount + 1))*PI + 0.5f*PI;
        for(u32 Meridian = 0;
            Meridian < MeridianCount;
            Meridian++)
        {
            r32 Theta = ((r32)Meridian / MeridianCount) * 2.0f*PI;
            r32 X = -sinf(Theta)*cosf(Phi);
            r32 Y = sinf(Phi);
            r32 Z = -cosf(Phi)*cosf(Theta);
            vec3 P = Radius*Normalize(vec3(X, Y, Z));
            Vertices.push_back(P);
            Vertices.push_back(Normalize(P));
        }
    }
    Vertices.push_back(Radius*vec3(0.0f, -1.0f, 0.0f));
    Vertices.push_back(vec3(0.0f, -1.0f, 0.0f));

    for(u32 Meridian = 0;
        Meridian < MeridianCount;
        Meridian++)
    {
        Indices.push_back(0);
        Indices.push_back(Meridian + 1);
        Indices.push_back(((Meridian + 1) % MeridianCount) + 1);
    }

    for(u32 Parallel = 0;
        Parallel < ParallelCount - 1;
        Parallel++)
    {
        for(u32 Meridian = 0;
            Meridian < MeridianCount;
            Meridian++)
        {
            u32 A = (Parallel*MeridianCount + 1) + Meridian;
            u32 B = ((Parallel + 1)*MeridianCount + 1) + Meridian;
            u32 C = ((Parallel + 1)*MeridianCount + 1) + ((Meridian + 1) % MeridianCount);
            u32 D = (Parallel*MeridianCount + 1) + ((Meridian + 1) % MeridianCount);

            Indices.push_back(A);
            Indices.push_back(B);
            Indices.push_back(C);
            Indices.push_back(A);
            Indices.push_back(C);
            Indices.push_back(D);
        }
    }

    for(u32 Meridian = 0;
        Meridian < MeridianCount;
        Meridian++)
    {
        Indices.push_back(((ParallelCount - 1)*MeridianCount + 1) + ((Meridian + 1) % MeridianCount));
        Indices.push_back(((ParallelCount - 1)*MeridianCount + 1) + Meridian);
        Indices.push_back((Vertices.size()/2) - 1);
    }
}

static u32 WindowWidth;
static u32 WindowHeight;
static void
GLFWFramebufferSizeCallback(GLFWwindow *Window, int NewWidth, int NewHeight)
{
    WindowWidth = NewWidth;
    WindowHeight = NewHeight;
    glViewport(0, 0, NewWidth, NewHeight);
}

int main(void)
{
	srand(1337);

	glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
	
    WindowWidth = 900; WindowHeight = 540;
    GLFWwindow *Window = glfwCreateWindow(WindowWidth, WindowHeight, "BulletDemo", 0, 0);
	glfwMakeContextCurrent(Window);
    glfwSwapInterval(1);
    glfwSetFramebufferSizeCallback(Window, GLFWFramebufferSizeCallback);

    GLFWmonitor *Monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode *VidMode = glfwGetVideoMode(Monitor);
    r32 TargetSecondsPerFrame = 1.0f / VidMode->refreshRate;

	glewInit();

    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(0.2f, 0.4f, 0.8f, 1.0f);

    shader DefaultShader("shaders/vertex.vert", "shaders/fragment.frag");
    shader ShadowMapShader("shaders/shadow.vert", "shaders/shadow.frag");

    r32 CubeVertices[] = 
    {
        // back face
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f, 
        // front face
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f, 
        // left face
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 
        // right face
        0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 
        0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 
        0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 
        0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f, 
        0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 
        0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f, 
        // bottom face
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 
        0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 
        0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 
        0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f, 
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f, 
        // top face
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 
        0.5f,  0.5f , 0.5f,  0.0f,  1.0f,  0.0f, 
        0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 
        0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f, 
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f, 
    };

    GLuint BoxVAO, BoxVBO, BoxInstanceColors, BoxInstanceMatrices;
    glGenVertexArrays(1, &BoxVAO);
    glGenBuffers(1, &BoxVBO);
    glGenBuffers(1, &BoxInstanceColors);
    glGenBuffers(1, &BoxInstanceMatrices);
    glBindVertexArray(BoxVAO);

    glBindBuffer(GL_ARRAY_BUFFER, BoxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(CubeVertices), CubeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(r32), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(r32), (void*)(3 * sizeof(r32)));

    glBindBuffer(GL_ARRAY_BUFFER, BoxInstanceColors);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(r32), (void *)0);
    glVertexAttribDivisor(2, 1);

    glBindBuffer(GL_ARRAY_BUFFER, BoxInstanceMatrices);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(1*sizeof(vec4)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(2*sizeof(vec4)));
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(3*sizeof(vec4)));
    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    glVertexAttribDivisor(5, 1);
    glVertexAttribDivisor(6, 1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    std::vector<vec3> SphereVertices;
    std::vector<u32> SphereIndices;
    GetSphereVertices(SphereVertices, SphereIndices, 20, 20);

    GLuint SphereVAO, SphereVBO, SphereEBO, SphereInstanceColors, SphereInstanceMatrices;
    glGenVertexArrays(1, &SphereVAO);
    glGenBuffers(1, &SphereVBO);
    glGenBuffers(1, &SphereEBO);
    glGenBuffers(1, &SphereInstanceColors);
    glGenBuffers(1, &SphereInstanceMatrices);
    glBindVertexArray(SphereVAO);

    glBindBuffer(GL_ARRAY_BUFFER, SphereVBO);
    glBufferData(GL_ARRAY_BUFFER, SphereVertices.size()*sizeof(vec3), &SphereVertices[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(r32), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(r32), (void*)(3 * sizeof(r32)));

    glBindBuffer(GL_ARRAY_BUFFER, SphereInstanceColors);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(r32), (void *)0);
    glVertexAttribDivisor(2, 1);

    glBindBuffer(GL_ARRAY_BUFFER, SphereInstanceMatrices);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)0);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(1*sizeof(vec4)));
    glEnableVertexAttribArray(5);
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(2*sizeof(vec4)));
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, 4*sizeof(vec4), (void *)(3*sizeof(vec4)));
    glVertexAttribDivisor(3, 1);
    glVertexAttribDivisor(4, 1);
    glVertexAttribDivisor(5, 1);
    glVertexAttribDivisor(6, 1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, SphereEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, SphereIndices.size()*sizeof(u32), &SphereIndices[0], GL_STATIC_DRAW);

    glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    GLuint ShadowFBO, ShadowMap;
    u32 ShadowMapWidth, ShadowMapHeight;
    ShadowMapWidth = ShadowMapHeight = 2048;
    glGenFramebuffers(1, &ShadowFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, ShadowFBO);
    glGenTextures(1, &ShadowMap);
    glBindTexture(GL_TEXTURE_2D, ShadowMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, ShadowMapWidth, ShadowMapHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, ShadowMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    vec2 ShadowSamplesOffsets[16];
    for(u32 Y = 0; Y < 4; Y++)
    {
        r32 StrataY1 = (Y / 4.0f);
        r32 StrataY2 = ((Y + 1) / 4.0f);

        for(u32 X = 0; X < 4; X++)
        {
            r32 StrataX1 = (X / 4.0f);
            r32 StrataX2 = ((X + 1) / 4.0f);

            r32 U = StrataX1 + GetRandomR32Between(0.0f, 1.0f)*(StrataX2 - StrataX1);
            r32 V = StrataY1 + GetRandomR32Between(0.0f, 1.0f)*(StrataY2 - StrataY1);

            vec2 DiskP = vec2(sqrtf(V)*cosf(2.0f*PI*U), sqrtf(V)*sinf(2.0f*PI*U));
            ShadowSamplesOffsets[Y*4 + X] = DiskP;
        }
    }

    GLuint ShadowNoiseTexture;
    vec2 ShadowNoise[256];
    for(u32 NoiseIndex = 0;
        NoiseIndex < ArrayCount(ShadowNoise);
        NoiseIndex++)
    {
        r32 Angle = GetRandomR32Between(0.0f, 2.0f*PI);
        r32 X = cosf(Angle);
        r32 Y = sinf(Angle);
        ShadowNoise[NoiseIndex] = Normalize(vec2(X, Y));
    }
    glGenTextures(1, &ShadowNoiseTexture);
    glBindTexture(GL_TEXTURE_2D, ShadowNoiseTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, 16, 16, 0, GL_RG, GL_FLOAT, ShadowNoise);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    physics Physics;
    Physics.Initialize();

	btCollisionShape* PlaneCollisionShape = new btStaticPlaneShape(btVector3(0.0f, 1.0f, 0.0f), 0.0f);
    btCollisionShape* BoxCollisionShape = new btBoxShape(btVector3(0.75f, 0.75f, 0.75f));
	btCollisionShape* SphereCollisionShape = new btSphereShape(btScalar(0.5f));

	mat4 GroundTransform = Translation(vec3(0.0f, -1.0f, 0.0f));
    rigid_body GroundRigidBody = Physics.CreateRigidBody(RigidBody_Plane, 0.0f, PlaneCollisionShape, 0.3f, GroundTransform, vec3(0.66f, 0.66f, 0.66f));
    Physics.AddRigidBody(GroundRigidBody);

    r32 LastFrameTime = (r32)glfwGetTime();
	u32 FrameIndex = 0;
    while(!glfwWindowShouldClose(Window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		if((FrameIndex % 10) == 0)
		{
            vec3 RandomRotationAxis = vec3(GetRandomR32Between(0.0f, 1.0f), GetRandomR32Between(0.0f, 1.0f), GetRandomR32Between(0.0f, 1.0f));
            r32 RandomRotationAngle = GetRandomR32Between(0.0f, 360.0f);

            vec3 RandomLinearVelocity = Hadamard(vec3(4.0f, 20.0f, 4.0f), 
                                                    vec3(GetRandomR32Between(-1.0f, 1.0f), GetRandomR32Between(0.0f, 1.0f), GetRandomR32Between(0.0f, 1.0f)));
            vec3 RandomAngularVelocity = vec3(GetRandomR32Between(0.0f, 10.0f), GetRandomR32Between(0.0f, 10.0f), GetRandomR32Between(0.0f, 10.0f));

            rigid_body RigidBody;
            r32 Mass = 5.0f;
            r32 Restitution = 0.3f;
            mat4 Transform = Translation(vec3(0.0f, 3.0f, 0.0f)) * 
                             Rotation(RandomRotationAngle, RandomRotationAxis);
            vec3 Color = vec3(GetRandomR32Between(0.3f, 1.0f), GetRandomR32Between(0.3f, 1.0f), GetRandomR32Between(0.3f, 1.0f));
            if((FrameIndex % 20) == 0)
            {
                RigidBody = Physics.CreateRigidBody(RigidBody_Box, Mass, BoxCollisionShape, Restitution, Transform, Color);
            }
            else
            {
                RigidBody = Physics.CreateRigidBody(RigidBody_Sphere, Mass, SphereCollisionShape, Restitution, Transform, Color);
            }

            RigidBody.BulletRigidBody->setLinearVelocity(btVector3(RandomLinearVelocity.x, RandomLinearVelocity.y, RandomLinearVelocity.z));
            RigidBody.BulletRigidBody->setAngularVelocity(btVector3(RandomAngularVelocity.x, RandomAngularVelocity.y, RandomAngularVelocity.z));
            RigidBody.BulletRigidBody->setRestitution(0.3f);
            RigidBody.BulletRigidBody->setDamping(0.1f, 0.1f);

            Physics.AddRigidBody(RigidBody);
		}
		FrameIndex++;

		std::cout << "RigidBodies count: " << Physics.RigidBodies.size() << std::endl;
        r32 PhysicsStart = (r32)glfwGetTime();
        Physics.Update(TargetSecondsPerFrame);
        std::cout << "PhysicsUpdate time: " << glfwGetTime() - PhysicsStart << std::endl;

        // NOTE(georgy): Shadow map render pass
        glBindFramebuffer(GL_FRAMEBUFFER, ShadowFBO);
		glClear(GL_DEPTH_BUFFER_BIT);
        glViewport(0, 0, ShadowMapWidth, ShadowMapHeight);
        mat4 LightProjection = Ortho(-30.0f, 60.0f, -50.0f, 50.0f, -15.0f, 50.0f);
        mat4 LightView = LookAt(vec3(3.0, 6.0, 3.0), vec3(0.0f, 0.0f, 0.0f));
        ShadowMapShader.Use();
        ShadowMapShader.SetMat4("Projection", LightProjection);
        ShadowMapShader.SetMat4("View", LightView);

        u32 BoxCount = 0;
        vec3 BoxColors[512];
        mat4 BoxMatrices[512];
        u32 SphereCount = 0;
        vec3 SphereColors[512];
        mat4 SphereMatrices[512];
        for(u32 RigidBodyIndex = 0;
            RigidBodyIndex < Physics.RigidBodies.size();
            RigidBodyIndex++)
        {
            btRigidBody *RigidBody = Physics.RigidBodies[RigidBodyIndex].BulletRigidBody;
			btTransform Transform;
			RigidBody->getMotionState()->getWorldTransform(Transform);
            mat4 Model;
            Transform.getOpenGLMatrix(Model.E);

            vec3 Color = Physics.RigidBodies[RigidBodyIndex].Color;

			switch(Physics.RigidBodies[RigidBodyIndex].Type)
			{
				case RigidBody_Plane:
				{
                    BoxColors[BoxCount] = Color;
                    BoxMatrices[BoxCount] = Model * Scaling(vec3(100.0f, 0.001f, 100.0f));
                    BoxCount++;
				} break;

				case RigidBody_Box:
				{
					BoxColors[BoxCount] = Color;
                    BoxMatrices[BoxCount] = Model * Scaling(1.5f);
                    BoxCount++;
				} break;

				case RigidBody_Sphere:
				{
                    SphereColors[SphereCount] = Color;
                    SphereMatrices[SphereCount] = Model;
                    SphereCount++;
				} break;
			}
        }
        glBindVertexArray(BoxVAO);
        glBindBuffer(GL_ARRAY_BUFFER, BoxInstanceColors);
        glBufferData(GL_ARRAY_BUFFER, BoxCount*sizeof(vec3), BoxColors, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, BoxInstanceMatrices);
        glBufferData(GL_ARRAY_BUFFER, BoxCount*sizeof(mat4), BoxMatrices, GL_STATIC_DRAW);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 36, BoxCount);
        glBindVertexArray(0);

        glBindVertexArray(SphereVAO);
        glBindBuffer(GL_ARRAY_BUFFER, SphereInstanceColors);
        glBufferData(GL_ARRAY_BUFFER, SphereCount*sizeof(vec3), SphereColors, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, SphereInstanceMatrices);
        glBufferData(GL_ARRAY_BUFFER, SphereCount*sizeof(mat4), SphereMatrices, GL_STATIC_DRAW);
        glDrawElementsInstanced(GL_TRIANGLES, SphereIndices.size(), GL_UNSIGNED_INT, 0, SphereCount);
        glBindVertexArray(0);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, WindowWidth, WindowHeight);

        // NOTE(georgy): Main render pass
        vec3 CameraWorldP = vec3(0.0f, 4.0f, 30.0f);
        mat4 Projection = Perspective(45.0f, (r32)WindowWidth / WindowHeight, 0.1f, 100.0f);
        mat4 View = LookAt(CameraWorldP, vec3(0.0f, 0.0f, 0.0f));
        DefaultShader.Use();
        DefaultShader.SetMat4("Projection", Projection);
        DefaultShader.SetMat4("View", View);
        DefaultShader.SetVec3("CameraWorldP", CameraWorldP);
        DefaultShader.SetMat4("LightViewProjection", LightProjection*LightView);
        DefaultShader.SetI32("ShadowMap", 0);
        DefaultShader.SetI32("ShadowNoiseTexture", 1);
        DefaultShader.SetVec2Array("SampleKernel", 16, ShadowSamplesOffsets);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ShadowMap);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, ShadowNoiseTexture);

        glBindVertexArray(BoxVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 36, BoxCount);
        glBindVertexArray(SphereVAO);
        glDrawElementsInstanced(GL_TRIANGLES, SphereIndices.size(), GL_UNSIGNED_INT, 0, SphereCount);
		glBindVertexArray(0);

        glfwPollEvents();
        glfwSwapBuffers(Window);

        r32 EndFrameTime = (r32)glfwGetTime();
        std::cout << "Frame time: " << EndFrameTime - LastFrameTime << "\n\n";
        LastFrameTime = EndFrameTime;
    }

    glfwTerminate();

    return(0);
}