// PhysicsWorld.h — the engine core.
//
// Owns the full Bullet pipeline (collision configuration, narrow-phase
// dispatcher, broad-phase, constraint solver, and the discrete dynamics world),
// plus every body and constraint we spawn. This is the "simulating" half of the
// modular framework: spawn bodies, wire constraints, and step the world forward
// with adjustable time-steps and solver substeps.
#pragma once

#include "Common.h"

#include <unordered_map>
#include <vector>

namespace pbx {

class PhysicsWorld {
public:
    explicit PhysicsWorld(const WorldConfig& cfg = {});
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&)            = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // --- spawning ------------------------------------------------------------
    // Spawn a body from a descriptor. Returns a stable integer id.
    int spawn(const BodyDesc& d);

    // --- constraints (joints) ------------------------------------------------
    // Pin a point on body A to a point on body B (or to world if b < 0).
    int addPoint2Point(int a, const btVector3& pivotA,
                        int b, const btVector3& pivotB);
    // Hinge about an axis; pass b < 0 to hinge against the world.
    int addHinge(int a, const btVector3& pivotA, const btVector3& axisA,
                 int b, const btVector3& pivotB, const btVector3& axisB);
    // Rigidly weld B to A at their current relative transform.
    int addFixed(int a, int b);
    // Slider along the frame X axis between A and B.
    int addSlider(int a, int b, const btTransform& frameInA,
                  const btTransform& frameInB, btScalar lower, btScalar upper);

    // --- stepping ------------------------------------------------------------
    // Advance one WorldConfig::timeStep. Returns wall-clock time in milliseconds.
    double step();
    void   stepN(int n);

    // Override step parameters at runtime (e.g. for stability sweeps).
    void setTimeStep(btScalar dt)  { cfg_.timeStep = dt; }
    void setSubSteps(int n)        { cfg_.subSteps = n; }
    void setGravity(const btVector3& g);

    // --- accessors -----------------------------------------------------------
    btDiscreteDynamicsWorld* world()       { return world_; }
    const WorldConfig&       config() const { return cfg_; }
    btRigidBody*             body(int id) const;
    const BodyRecord*        record(int id) const;
    const std::vector<std::unique_ptr<BodyRecord>>& bodies() const { return bodies_; }

    int   stepCount() const { return stepCount_; }
    double lastStepMs() const { return lastStepMs_; }

private:
    WorldConfig cfg_;

    // Bullet pipeline — declared in construction order; destroyed in reverse.
    btDefaultCollisionConfiguration*     collisionConfig_ = nullptr;
    btCollisionDispatcher*               dispatcher_      = nullptr;  // narrow phase
    btBroadphaseInterface*               broadphase_      = nullptr;  // broad phase
    btSequentialImpulseConstraintSolver* solver_          = nullptr;
    btDiscreteDynamicsWorld*             world_           = nullptr;

    std::vector<std::unique_ptr<BodyRecord>>   bodies_;
    std::unordered_map<int, BodyRecord*>       byId_;
    std::vector<std::unique_ptr<btTypedConstraint>> constraints_;

    int    nextId_    = 0;
    int    stepCount_ = 0;
    double lastStepMs_ = 0.0;
};

}  // namespace pbx
