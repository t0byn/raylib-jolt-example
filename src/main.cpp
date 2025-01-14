#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>

#include <Jolt/Physics/Constraints/HingeConstraint.h>

#include <Jolt/Renderer/DebugRenderer.h>

JPH_SUPPRESS_WARNINGS

using namespace JPH::literals;

// raylib includes
#include <raylib.h>
#include <rlgl.h>

JPH_MSVC_SUPPRESS_WARNING(4996)
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>

#define UNUSED_VAR(x) ((void)(x))

namespace FromJoltToRaylib
{
    inline Vector3 GetVector3(JPH::Vec3 v)
    {
        return Vector3{ v.GetX(), v.GetY(), v.GetZ() };
    }

    inline Vector3 GetVector3(JPH::Float3 v)
    {
        return Vector3{ v.x, v.y, v.z };
    }

    inline Color GetColor(JPH::Color clr)
    {
        return Color{ clr.r, clr.g, clr.b, clr.a };
    }

    inline Matrix GetMatrix(JPH::RMat44 mat)
    {
        Matrix matrix = {
            mat(0,0), mat(1,0), mat(2,0), mat(3,0),
            mat(0,1), mat(1,1), mat(2,1), mat(3,1),
            mat(0,2), mat(1,2), mat(2,2), mat(3,2),
            mat(0,3), mat(1,3), mat(2,3), mat(3,3),
        };
        return matrix;
    }
}

namespace FromRaylibToJolt
{
    inline JPH::Vec3 GetVec3(Vector3 v)
    {
        return JPH::Vec3(v.x, v.y, v.z);
    }

    inline JPH::RVec3 GetRVec3(Vector3 v)
    {
        return JPH::RVec3(v.x, v.y, v.z);
    }
}

namespace PhysObjectLayer
{
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

namespace PhysBPLayer
{
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS(2);
};

struct Box
{
    Vector3 extend;
    Color color;
    JPH::BodyID body;
};

Box CreateBox(Vector3 inPos, Vector3 inExtend, Color inColor, JPH::BodyInterface& physBodyInterface, JPH::EMotionType inMotionType = JPH::EMotionType::Static, 
    JPH::EActivation inActivationMode = JPH::EActivation::DontActivate, JPH::Vec3Arg inRotateAxis = JPH::Vec3(0,0,0), float inAngle = 0)
{
    Box box;
    box.extend = inExtend;
    box.color = inColor;

    JPH::Vec3 halfExtend(inExtend.x / 2, inExtend.y / 2, inExtend.z / 2);
    JPH::RVec3Arg pos(inPos.x, inPos.y, inPos.z);
    JPH::Quat quat;
    if (inAngle == 0)
        quat = JPH::Quat::sIdentity();
    else
    {
        JPH::Vec3Arg axis = inRotateAxis.Normalized();
        quat = JPH::Quat::sRotation(axis, inAngle);
    }
    JPH::ObjectLayer layer;
    if (inMotionType == JPH::EMotionType::Static) layer = PhysObjectLayer::NON_MOVING;
    else layer = PhysObjectLayer::MOVING;
    JPH::BodyCreationSettings settings(new JPH::BoxShape(halfExtend), pos, quat, inMotionType, layer);
    box.body = physBodyInterface.CreateAndAddBody(settings, inActivationMode);

    return box;
}

void DrawBox(Box inBox, JPH::BodyInterface& physBodyInterface)
{
    JPH::RMat44 mat = physBodyInterface.GetWorldTransform(inBox.body);
    JPH::Vec3 v = mat.GetTranslation();
    Matrix transform = FromJoltToRaylib::GetMatrix(mat);
    rlPushMatrix();
    rlMultMatrixf((float*)&transform);
    DrawCubeV({ 0,0,0 }, inBox.extend, inBox.color);
    rlPopMatrix();
}

struct Sphere
{
    float radius;
    Color color;
    JPH::BodyID body;
};

Sphere CreateSphere(Vector3 inPos, float inRadius, Color color, JPH::BodyInterface& physBodyInterface, JPH::EMotionType inMotionType = JPH::EMotionType::Static,
    JPH::EActivation inActivationMode = JPH::EActivation::DontActivate)
{
    Sphere sphere;
    sphere.radius = inRadius;
    sphere.color = color;

    JPH::RVec3 pos = FromRaylibToJolt::GetRVec3(inPos);
    JPH::Quat quat = JPH::Quat::sIdentity();
    JPH::ObjectLayer layer;
    if (inMotionType == JPH::EMotionType::Static) layer = PhysObjectLayer::NON_MOVING;
    else layer = PhysObjectLayer::MOVING;
    JPH::BodyCreationSettings settings(new JPH::SphereShape(inRadius), pos, quat, inMotionType, layer);
    sphere.body = physBodyInterface.CreateAndAddBody(settings, inActivationMode);

    return sphere;
}

void DrawSphere(Sphere sphere, JPH::BodyInterface& physBodyInterface)
{
    JPH::Mat44 mat = physBodyInterface.GetWorldTransform(sphere.body);
    Matrix transform = FromJoltToRaylib::GetMatrix(mat);
    rlPushMatrix();
    rlMultMatrixf((float*)&transform);
    DrawSphere({ 0,0,0 }, sphere.radius, sphere.color);
    rlPopMatrix();
}

void PhysTraceImpl(const char* inFMT, ...)
{
    va_list list;
    va_start(list, inFMT);
    vprintf(inFMT, list);
    va_end(list);
}

bool PhysAssertFailedImpl(const char* inExpression, const char* inMessage, const char* inFile, JPH::uint inLine)
{
    printf("%s:%ud:(%s)", inFile, inLine, inExpression);
    if (inMessage != NULL) printf("%s", inMessage);
    printf("\n");
    return true;
}

class PhysBPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
    PhysBPLayerInterfaceImpl()
    {
        mObjToBP[PhysObjectLayer::NON_MOVING] = PhysBPLayer::NON_MOVING;
        mObjToBP[PhysObjectLayer::MOVING] = PhysBPLayer::MOVING;
    }

    virtual JPH::uint GetNumBroadPhaseLayers() const override
    {
        return PhysBPLayer::NUM_LAYERS;
    }

    virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
    {
        return mObjToBP[inLayer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
    {
        switch ((JPH::BroadPhaseLayer::Type)inLayer)
        {
        case (JPH::BroadPhaseLayer::Type)PhysBPLayer::NON_MOVING: return "NON_MOVING";
        case (JPH::BroadPhaseLayer::Type)PhysBPLayer::MOVING: return "MOVING";
        default: JPH_ASSERT(false); return "INVALID";
        }
    }
#endif

private:
    JPH::BroadPhaseLayer mObjToBP[PhysObjectLayer::NUM_LAYERS];
};

class PhysObjectVsBPLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case PhysObjectLayer::NON_MOVING: return inLayer2 == PhysBPLayer::MOVING;
        case PhysObjectLayer::MOVING: return true;
        default: JPH_ASSERT(false); return false;
        }
    }
};

class PhysObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
    virtual bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override
    {
        switch (inLayer1)
        {
        case PhysObjectLayer::NON_MOVING: return inLayer2 == PhysObjectLayer::MOVING;
        case PhysObjectLayer::MOVING: return true;
        default: JPH_ASSERT(false); return false;
        }
    }
};

class PhysBodyActivationListener : public JPH::BodyActivationListener
{
    virtual void OnBodyActivated(const JPH::BodyID& inBodyID, JPH::uint64 inBodyUserData) override
    {
        printf("Body %X got activated.\n", inBodyID.GetIndexAndSequenceNumber());
    }

    virtual void OnBodyDeactivated(const JPH::BodyID& inBodyID, JPH::uint64 inBodyUserData) override
    {
        printf("Body %X got deactivated.\n", inBodyID.GetIndexAndSequenceNumber());
    }
};

class PhysContactListener : public JPH::ContactListener
{
    virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override
    {
        printf("Contact validate callback\n");
        return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
    }

    virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
    {
        printf("A contact was added\n");
    }

    virtual void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2, const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
    {
        printf("A contact was persisted\n");
    }

    virtual void OnContactRemoved(const JPH::SubShapeIDPair& inSubShapePair) override
    {
        printf("A contact was removed\n");
    }
};

class PhysDebugRenderer final : public JPH::DebugRenderer
{
public:
    PhysDebugRenderer() : DebugRenderer() { Initialize(); };
    virtual ~PhysDebugRenderer() {};

    void SetCameraPos(Vector3 inCameraPos)
    {
        mCameraPos = FromRaylibToJolt::GetRVec3(inCameraPos);
    }

	/// Draw line
    virtual void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override
    {
        Vector3 from = FromJoltToRaylib::GetVector3(inFrom);
        Vector3 to = FromJoltToRaylib::GetVector3(inTo);
        Color color = FromJoltToRaylib::GetColor(inColor);
        DrawLine3D(from, to, color);
    }

	/// Draw a single back face culled triangle
    virtual void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3, JPH::ColorArg inColor, ECastShadow inCastShadow = ECastShadow::Off) override
    {
        Vector3 v1 = FromJoltToRaylib::GetVector3(inV1);
        Vector3 v2 = FromJoltToRaylib::GetVector3(inV2);
        Vector3 v3 = FromJoltToRaylib::GetVector3(inV3);
        Color color = FromJoltToRaylib::GetColor(inColor);
        UNUSED_VAR(inCastShadow); /// TODO: cast shadow?
        DrawTriangle3D(v1, v2, v3, color);
    }

	/// Create a batch of triangles that can be drawn efficiently
    virtual Batch CreateTriangleBatch(const Triangle* inTriangles, int inTriangleCount) override
    {
        BatchImpl* batch = new BatchImpl;
        if (inTriangles == nullptr || inTriangleCount == 0)
            return batch;

        batch->mTriangles.assign(inTriangles, inTriangles + inTriangleCount);
        return batch;
    }
    virtual Batch CreateTriangleBatch(const Vertex* inVertices, int inVertexCount, const JPH::uint32* inIndices, int inIndexCount) override
    {
        BatchImpl* batch = new BatchImpl;
        if (inVertices == nullptr || inVertexCount == 0 || inIndices == nullptr || inIndexCount == 0)
            return batch;

        // Convert indexed triangle list to triangle list
        batch->mTriangles.resize(inIndexCount / 3);
        for (size_t t = 0; t < batch->mTriangles.size(); ++t)
        {
            Triangle& triangle = batch->mTriangles[t];
            triangle.mV[0] = inVertices[inIndices[t * 3 + 0]];
            triangle.mV[1] = inVertices[inIndices[t * 3 + 1]];
            triangle.mV[2] = inVertices[inIndices[t * 3 + 2]];
        }

        return batch;
    }

	/// Draw some geometry
    virtual void DrawGeometry(JPH::RMat44Arg inModelMatrix, const JPH::AABox& inWorldSpaceBounds, float inLODScaleSq, JPH::ColorArg inModelColor, const GeometryRef& inGeometry, ECullMode inCullMode = ECullMode::CullBackFace, ECastShadow inCastShadow = ECastShadow::On, EDrawMode inDrawMode = EDrawMode::Solid) override
    {
        // Figure out which LOD to use
        const LOD* lod = inGeometry->mLODs.data();
        lod = &inGeometry->GetLOD(JPH::Vec3(mCameraPos), inWorldSpaceBounds, inLODScaleSq);

        // Draw the batch
        const BatchImpl* batch = static_cast<const BatchImpl*>(lod->mTriangleBatch.GetPtr());
        for (const Triangle& triangle : batch->mTriangles)
        {
            JPH::RVec3 v0 = inModelMatrix * JPH::Vec3(triangle.mV[0].mPosition);
            JPH::RVec3 v1 = inModelMatrix * JPH::Vec3(triangle.mV[1].mPosition);
            JPH::RVec3 v2 = inModelMatrix * JPH::Vec3(triangle.mV[2].mPosition);
            Color color;

            switch (inDrawMode)
            {
            case EDrawMode::Wireframe:
                rlBegin(RL_LINES);
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[0].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v0.GetX(), v0.GetY(), v0.GetZ());
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[1].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v1.GetX(), v1.GetY(), v1.GetZ());

                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[1].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v1.GetX(), v1.GetY(), v1.GetZ());
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[2].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v2.GetX(), v2.GetY(), v2.GetZ());

                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[2].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v2.GetX(), v2.GetY(), v2.GetZ());
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[0].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v0.GetX(), v0.GetY(), v0.GetZ());
                rlEnd();
                break;

            case EDrawMode::Solid:
                rlBegin(RL_TRIANGLES);
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[0].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v0.GetX(), v0.GetY(), v0.GetZ());
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[1].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v1.GetX(), v1.GetY(), v1.GetZ());
                    color = FromJoltToRaylib::GetColor(inModelColor * triangle.mV[2].mColor);
                    rlColor4ub(color.r, color.g, color.b, color.a);
                    rlVertex3f(v2.GetX(), v2.GetY(), v2.GetZ());
                rlEnd();
                break;
            }
        }
    }

	/// Draw text
    virtual void DrawText3D(JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor = JPH::Color::sWhite, float inHeight = 0.5f) override
    {
        /// TODO: https://www.raylib.com/examples/text/loader.html?name=text_draw_3d
        printf("DrawText3D: %s\n", inString.data());
    }

private:
	/// Implementation specific batch object
	class BatchImpl : public JPH::RefTargetVirtual
	{
	public:
		JPH_OVERRIDE_NEW_DELETE

		virtual void AddRef() override { ++mRefCount; }
		virtual void Release() override { if (--mRefCount == 0) delete this; }

		JPH::Array<Triangle> mTriangles;

	private:
		std::atomic<JPH::uint32> mRefCount = 0;
	};

    JPH::RVec3 mCameraPos;
};

int main(int argc, char** argv)
{
    UNUSED_VAR(argc);
    UNUSED_VAR(argv);

    InitWindow(1920, 1080, "Example");

    SetTargetFPS(60);

    Camera3D camera;
    camera.position = { 0, 90, -40 };
    camera.fovy = 90;
    camera.target = { 0, 0, 0 };
    camera.up = { 0, 1, 0 };
    camera.projection = CAMERA_PERSPECTIVE;


    JPH::RegisterDefaultAllocator();

    JPH::Trace = PhysTraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = PhysAssertFailedImpl);

    JPH::Factory::sInstance = new JPH::Factory();

    JPH::RegisterTypes();

    JPH::TempAllocatorImpl tempAllocator(10 * 1024 * 1024);

    JPH::JobSystemThreadPool jobSystem(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, std::thread::hardware_concurrency() - 1);

    const JPH::uint cMaxBodies = 1024;
    const JPH::uint cNumBodyMutexes = 0;
    const JPH::uint cMaxBodyPairs = 1024;
    const JPH::uint cMaxContactConstraints = 1024;

    PhysBPLayerInterfaceImpl bpLayerInterface;
    PhysObjectVsBPLayerFilterImpl objBpLayerFilter;
    PhysObjectLayerPairFilterImpl objLayerPairFilter;

    JPH::PhysicsSystem physSystem;
    physSystem.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
        bpLayerInterface, objBpLayerFilter, objLayerPairFilter);

    JPH::PhysicsSettings physSetting;
    physSystem.SetPhysicsSettings(physSetting);

    PhysBodyActivationListener bodyActivationListener;
    physSystem.SetBodyActivationListener(&bodyActivationListener);

    PhysContactListener contactListener;
    physSystem.SetContactListener(&contactListener);

    JPH::BodyInterface& physBodyInterface = physSystem.GetBodyInterface();

    Box floor = CreateBox({ 0, 0, 0 }, { 100, 2, 100 }, GREEN, physBodyInterface);
    Box wall1 = CreateBox({ 0, 16, 51 }, { 100, 30, 2 }, GREEN, physBodyInterface);
    Box wall2 = CreateBox({ 0, 16, -51 }, { 100, 30, 2 }, GREEN, physBodyInterface);
    Box wall3 = CreateBox({ 51, 16, 0 }, { 100, 30, 2 }, GREEN, physBodyInterface, JPH::EMotionType::Static, JPH::EActivation::DontActivate, JPH::Vec3(0, 1, 0), PI/2);
    Box wall4 = CreateBox({ -51, 16, 0 }, { 100, 30, 2 }, GREEN, physBodyInterface, JPH::EMotionType::Static, JPH::EActivation::DontActivate, JPH::Vec3(0, 1, 0), -PI/2);

    Sphere sphere1 = CreateSphere({ 0, 36, 0 }, 6, RED, physBodyInterface, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    physBodyInterface.SetLinearVelocity(sphere1.body, JPH::Vec3(5, 0, 0));

    Sphere sphere2 = CreateSphere({ 0, 36, 10 }, 6, RED, physBodyInterface, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    physBodyInterface.SetLinearVelocity(sphere2.body, JPH::Vec3(5, 0, 5));

    Sphere sphere3 = CreateSphere({ 0, 36, -10 }, 6, RED, physBodyInterface, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    physBodyInterface.SetLinearVelocity(sphere3.body, JPH::Vec3(-5, 0, -5));

    Sphere sphere4 = CreateSphere({ 0, 42, 0 }, 6, RED, physBodyInterface, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    physBodyInterface.SetLinearVelocity(sphere3.body, JPH::Vec3(-5, 0, 0));

    Box fan = CreateBox({ 0, 11, 0 }, { 100, 20, 10 }, BLUE, physBodyInterface, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    {
        const JPH::BodyLockInterface& physBodyLockInterface = physSystem.GetBodyLockInterface();
        JPH::BodyLockWrite lock(physBodyLockInterface, fan.body);
        if (lock.Succeeded())
        {
            JPH::Body& body = lock.GetBody();
            body.GetMotionProperties()->SetLinearDamping(0);
            body.GetMotionProperties()->SetAngularDamping(0);
        }
    }

    JPH::HingeConstraintSettings hingeSettings;
    hingeSettings.mPoint1 = JPH::RVec3(0, 0, 0);
    hingeSettings.mPoint2 = JPH::RVec3(0, 1, 0);

    JPH::HingeConstraint* hingeConstraint;
    {
        const JPH::BodyLockInterface& physBodyLockInterface = physSystem.GetBodyLockInterface();
        JPH::BodyID hingeBodies[2] = { floor.body, fan.body };
        JPH::BodyLockMultiWrite lock(physBodyLockInterface, hingeBodies, 4);
        hingeConstraint = (JPH::HingeConstraint*)hingeSettings.Create(*(lock.GetBody(0)), *(lock.GetBody(1)));
    }
    JPH_ASSERT(hingeConstraint != nullptr);
    hingeConstraint->SetMotorState(JPH::EMotorState::Velocity);
    hingeConstraint->SetTargetAngularVelocity(2);

    physSystem.AddConstraint(hingeConstraint);

    const float cDeltaTime = 1.0f / 60.0f;
    const int cCollisionSteps = 1;

    physSystem.OptimizeBroadPhase();

    PhysDebugRenderer physDebugRenderer;
    JPH::BodyManager::DrawSettings drawSettings;
    drawSettings.mDrawShape = true;

    bool drawObjects = true;
    bool drawBodies = false;
    bool drawConstraints = false;

    bool showDrawSettings = false;

    JPH::uint step = 0;
    while (!WindowShouldClose())
    {
        if (physBodyInterface.IsActive(sphere1.body))
        {
            ++step;

            JPH::RVec3 position = physBodyInterface.GetCenterOfMassPosition(sphere1.body);
            JPH::Vec3 velocity = physBodyInterface.GetLinearVelocity(sphere1.body);
            printf("Step %ud: Position = (%f, %f, %f), Velocity = (%f, %f, %f)\n",
                step, position.GetX(), position.GetY(), position.GetZ(),
                velocity.GetX(), velocity.GetY(), velocity.GetZ());
        }

        physSystem.Update(cDeltaTime, cCollisionSteps, &tempAllocator, &jobSystem);

        physDebugRenderer.SetCameraPos(camera.position);

        BeginDrawing();

        ClearBackground(BLACK);

        BeginMode3D(camera);

        DrawGrid(20, 10);

        if (drawObjects)
        {
            DrawBox(floor, physBodyInterface);
            DrawBox(wall1, physBodyInterface);
            DrawBox(wall2, physBodyInterface);
            DrawBox(wall3, physBodyInterface);
            DrawBox(wall4, physBodyInterface);

            DrawBox(fan, physBodyInterface);

            DrawSphere(sphere1, physBodyInterface);
            DrawSphere(sphere2, physBodyInterface);
            DrawSphere(sphere3, physBodyInterface);
            DrawSphere(sphere4, physBodyInterface);
        }
        if (drawBodies)
        {
            physSystem.DrawBodies(drawSettings, &physDebugRenderer);
        }
        if (drawConstraints)
        {
            physSystem.DrawConstraints(&physDebugRenderer);
        }

        EndMode3D();

        GuiCheckBox({ 16, 16, 16, 16 }, "DrawObjects", &drawObjects);
        GuiCheckBox({ 16, 32, 16, 16 }, "DrawBodies", &drawBodies);
        GuiCheckBox({ 16, 48, 16, 16 }, "DrawConstraints", &drawConstraints);

        if (GuiButton({ (float)GetScreenWidth() - 128, (float)GetScreenHeight() - 32, 128, 32 }, "DrawSettings"))
        {
            showDrawSettings = !showDrawSettings;
        }

        if (showDrawSettings)
        {
            if (GuiWindowBox({ (float)GetScreenWidth() / 2 - 128, (float)GetScreenHeight() / 2 - 256 , 256, 512 }, "DrawSetting"))
            {
                showDrawSettings = false;
            }
            else
            {
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 8, 16, 16 }, 
                    "DrawGetSupportFunction", &drawSettings.mDrawGetSupportFunction);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 24, 16, 16 }, 
                    "DrawSupportDirection", &drawSettings.mDrawSupportDirection);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 40, 16, 16 }, 
                    "DrawGetSupportingFace", &drawSettings.mDrawGetSupportingFace);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 56, 16, 16 }, 
                    "DrawShape", &drawSettings.mDrawShape);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 72, 16, 16 }, 
                    "DrawShapeWireframe", &drawSettings.mDrawShapeWireframe);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 88, 16, 16 }, 
                    "DrawBoundingBox", &drawSettings.mDrawBoundingBox);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 104, 16, 16 }, 
                    "DrawCenterOfMassTransform", &drawSettings.mDrawCenterOfMassTransform);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 120, 16, 16 }, 
                    "DrawWorldTransform", &drawSettings.mDrawWorldTransform);
                GuiCheckBox({ (float)GetScreenWidth() / 2 - 128 + 8, (float)GetScreenHeight() / 2 - 256 + RAYGUI_WINDOWBOX_STATUSBAR_HEIGHT + 136, 16, 16 }, 
                    "DrawVelocity", &drawSettings.mDrawVelocity);
            }
        }

        EndDrawing();
    }

    return 0;
}