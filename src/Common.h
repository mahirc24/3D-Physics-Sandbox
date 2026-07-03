// Common.h — shared types for the physics sandbox.
//
// Everything the rest of the engine needs to describe an object, a world, and a
// query result lives here. Keeping these in one small header makes the modular
// spawn/config framework easy to read: a scenario just fills in BodyDesc structs.
#pragma once

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <string>

namespace pbx {

// ---- Shapes we know how to build -------------------------------------------
enum class ShapeType {
    Box,       // halfExtents
    Sphere,    // radius
    Capsule,   // radius, height (along Y)
    Cylinder,  // radius, height (along Y)
    Plane,     // planeNormal, planeConstant (static only)
    Ground     // convenience: infinite plane, normal +Y, constant 0
};

// ---- Description of a body ---------------------------------------------------
// A pure-data struct. Scenarios and the JSON-free scene builders populate these
// and hand them to the factory. Sensible defaults keep call sites terse.
struct BodyDesc {
    std::string name = "body";
    ShapeType   type = ShapeType::Box;

    // Shape parameters (only the ones relevant to `type` are read).
    btVector3 halfExtents = btVector3(0.5f, 0.5f, 0.5f);  // Box
    btScalar  radius      = 0.5f;                         // Sphere/Capsule/Cylinder
    btScalar  height      = 1.0f;                         // Capsule/Cylinder
    btVector3 planeNormal = btVector3(0, 1, 0);           // Plane
    btScalar  planeConstant = 0.0f;                       // Plane

    // Mass. 0 => static (immovable, infinite mass).
    btScalar mass = 1.0f;

    // Initial state.
    btVector3    position    = btVector3(0, 0, 0);
    btQuaternion orientation = btQuaternion(0, 0, 0, 1);
    btVector3    linearVelocity  = btVector3(0, 0, 0);
    btVector3    angularVelocity = btVector3(0, 0, 0);

    // Material / damping.
    btScalar friction         = 0.5f;
    btScalar restitution      = 0.0f;   // bounciness [0,1]
    btScalar rollingFriction  = 0.0f;
    btScalar spinningFriction = 0.0f;
    btScalar linearDamping    = 0.0f;
    btScalar angularDamping   = 0.0f;

    // Continuous collision detection — stops thin/fast bodies tunnelling.
    bool     ccd = false;
    btScalar ccdMotionThreshold  = 0.0f;  // if 0 and ccd, auto-derived from radius
    btScalar ccdSweptSphereRadius = 0.0f; // if 0 and ccd, auto-derived
};

// ---- Owning record for a live body ------------------------------------------
// Bullet requires the shape and motion state to outlive the rigid body, and the
// body to be removed from the world before any of it is freed. unique_ptr member
// order guarantees the body is destroyed first (reverse declaration order), and
// PhysicsWorld removes it from the world before the record is dropped.
struct BodyRecord {
    int         id   = -1;
    std::string name;
    btScalar    mass = 0;

    std::unique_ptr<btCollisionShape> shape;
    std::unique_ptr<btMotionState>    motion;
    std::unique_ptr<btRigidBody>      body;   // declared last -> destroyed first
};

// ---- World configuration -----------------------------------------------------
struct WorldConfig {
    btVector3 gravity   = btVector3(0, -9.81f, 0);
    btScalar  timeStep  = btScalar(1.0) / btScalar(60.0);  // sim seconds per step()
    int       subSteps  = 10;   // max internal solver substeps per step()
    // Internal fixed integration step. Smaller improves tunnelling/stack
    // behaviour, but Bullet's discrete restitution is most accurate around
    // 1/60–1/120; going much finer bleeds bounce energy into persistent-contact
    // resolution. 1/120 is a good default. Constraint: timeStep <= subSteps*fixedStep.
    btScalar  fixedStep = btScalar(1.0) / btScalar(120.0);
    int       solverIterations = 10;                       // constraint iterations
};

// ---- Ray query result --------------------------------------------------------
struct RayHit {
    bool      hit       = false;
    int       bodyId    = -1;
    btVector3 point     = btVector3(0, 0, 0);
    btVector3 normal    = btVector3(0, 0, 0);
    btScalar  fraction  = btScalar(1);   // 0=at `from`, 1=at `to`
    btScalar  distance  = btScalar(0);
};

}  // namespace pbx
