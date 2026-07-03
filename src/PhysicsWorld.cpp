#include "PhysicsWorld.h"

#include "RigidBodyFactory.h"

#include <chrono>

namespace pbx {

PhysicsWorld::PhysicsWorld(const WorldConfig& cfg) : cfg_(cfg) {
    // Narrow-phase collision configuration + dispatcher.
    collisionConfig_ = new btDefaultCollisionConfiguration();
    dispatcher_      = new btCollisionDispatcher(collisionConfig_);

    // Broad-phase: dynamic bounding-volume tree (good general-purpose choice).
    broadphase_ = new btDbvtBroadphase();

    // Sequential-impulse constraint solver handles contact + joint constraints.
    solver_ = new btSequentialImpulseConstraintSolver();

    world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_, solver_,
                                         collisionConfig_);
    world_->setGravity(cfg_.gravity);
    world_->getSolverInfo().m_numIterations = cfg_.solverIterations;
}

PhysicsWorld::~PhysicsWorld() {
    // Release a transient mouse-pick grab before tearing anything down.
    clearPick();

    // Remove & free constraints first (they reference bodies).
    for (auto& c : constraints_) {
        if (c) world_->removeConstraint(c.get());
    }
    constraints_.clear();

    // Remove bodies from the world before their records (and thus the Bullet
    // objects) are destroyed.
    for (auto& rec : bodies_) {
        if (rec && rec->body) world_->removeRigidBody(rec->body.get());
    }
    bodies_.clear();  // frees body -> motion -> shape per record, in that order

    // Tear down the pipeline in reverse construction order.
    delete world_;
    delete solver_;
    delete broadphase_;
    delete dispatcher_;
    delete collisionConfig_;
}

int PhysicsWorld::spawn(const BodyDesc& d) {
    const int id = nextId_++;
    auto rec = RigidBodyFactory::create(d, id);
    world_->addRigidBody(rec->body.get());
    byId_[id] = rec.get();
    bodies_.push_back(std::move(rec));
    return id;
}

btRigidBody* PhysicsWorld::body(int id) const {
    auto it = byId_.find(id);
    return it == byId_.end() ? nullptr : it->second->body.get();
}

const BodyRecord* PhysicsWorld::record(int id) const {
    auto it = byId_.find(id);
    return it == byId_.end() ? nullptr : it->second;
}

void PhysicsWorld::setGravity(const btVector3& g) {
    cfg_.gravity = g;
    world_->setGravity(g);
}

// --- constraints -------------------------------------------------------------
int PhysicsWorld::addPoint2Point(int a, const btVector3& pivotA,
                                 int b, const btVector3& pivotB) {
    btRigidBody* ra = body(a);
    if (!ra) return -1;
    btTypedConstraint* c = nullptr;
    if (b < 0) {
        c = new btPoint2PointConstraint(*ra, pivotA);
    } else {
        btRigidBody* rb = body(b);
        if (!rb) return -1;
        c = new btPoint2PointConstraint(*ra, *rb, pivotA, pivotB);
    }
    world_->addConstraint(c, /*disableCollisionsBetweenLinkedBodies=*/true);
    const int idx = static_cast<int>(constraints_.size());
    constraints_.emplace_back(c);
    return idx;
}

int PhysicsWorld::addHinge(int a, const btVector3& pivotA, const btVector3& axisA,
                           int b, const btVector3& pivotB, const btVector3& axisB) {
    btRigidBody* ra = body(a);
    if (!ra) return -1;
    btHingeConstraint* c = nullptr;
    if (b < 0) {
        btVector3 pa = pivotA, aa = axisA;
        c = new btHingeConstraint(*ra, pa, aa);
    } else {
        btRigidBody* rb = body(b);
        if (!rb) return -1;
        btVector3 pa = pivotA, pb = pivotB, aa = axisA, ab = axisB;
        c = new btHingeConstraint(*ra, *rb, pa, pb, aa, ab);
    }
    world_->addConstraint(c, true);
    const int idx = static_cast<int>(constraints_.size());
    constraints_.emplace_back(c);
    return idx;
}

int PhysicsWorld::addFixed(int a, int b) {
    btRigidBody* ra = body(a);
    btRigidBody* rb = body(b);
    if (!ra || !rb) return -1;

    // Build frames so the current relative pose is preserved.
    btTransform ta = ra->getWorldTransform();
    btTransform tb = rb->getWorldTransform();
    btTransform frameInA = ta.inverse() * tb;
    btTransform frameInB;
    frameInB.setIdentity();

    auto* c = new btFixedConstraint(*ra, *rb, frameInA, frameInB);
    world_->addConstraint(c, true);
    const int idx = static_cast<int>(constraints_.size());
    constraints_.emplace_back(c);
    return idx;
}

int PhysicsWorld::addSlider(int a, int b, const btTransform& frameInA,
                            const btTransform& frameInB, btScalar lower,
                            btScalar upper) {
    btRigidBody* ra = body(a);
    btRigidBody* rb = body(b);
    if (!ra || !rb) return -1;
    auto* c = new btSliderConstraint(*ra, *rb, frameInA, frameInB,
                                     /*useLinearReferenceFrameA=*/true);
    c->setLowerLinLimit(lower);
    c->setUpperLinLimit(upper);
    world_->addConstraint(c, true);
    const int idx = static_cast<int>(constraints_.size());
    constraints_.emplace_back(c);
    return idx;
}

// --- interactive mouse picking ----------------------------------------------
btPoint2PointConstraint* PhysicsWorld::createPickConstraint(
    int bodyId, const btVector3& worldPivot) {
    clearPick();
    btRigidBody* b = body(bodyId);
    if (!b) return nullptr;

    // Wake the body and stop it deactivating while grabbed.
    b->activate(true);
    b->setActivationState(DISABLE_DEACTIVATION);

    // Convert the world grab point into the body's local frame.
    const btVector3 localPivot = b->getCenterOfMassTransform().inverse() * worldPivot;
    auto* p2p = new btPoint2PointConstraint(*b, localPivot);
    // Soft settings so dragging feels springy instead of explosive.
    p2p->m_setting.m_impulseClamp = 30.0f;
    p2p->m_setting.m_tau          = 0.1f;

    world_->addConstraint(p2p, /*disableCollisions=*/false);
    pickConstraint_ = p2p;
    return p2p;
}

void PhysicsWorld::updatePick(const btVector3& worldPivot) {
    if (pickConstraint_) pickConstraint_->setPivotB(worldPivot);
}

void PhysicsWorld::clearPick() {
    if (!pickConstraint_) return;
    // Restore normal deactivation on the grabbed body.
    if (btRigidBody* b = &pickConstraint_->getRigidBodyA()) {
        b->forceActivationState(ACTIVE_TAG);
        b->activate(true);
    }
    world_->removeConstraint(pickConstraint_);
    delete pickConstraint_;
    pickConstraint_ = nullptr;
}

// --- stepping ----------------------------------------------------------------
double PhysicsWorld::step() {
    const auto t0 = std::chrono::high_resolution_clock::now();
    // stepSimulation(dt, maxSubSteps, fixedTimeStep): Bullet subdivides `dt`
    // into up to `maxSubSteps` internal fixed steps of `fixedTimeStep`, which is
    // what keeps stacked and fast-moving bodies numerically stable under load.
    world_->stepSimulation(cfg_.timeStep, cfg_.subSteps, cfg_.fixedStep);
    const auto t1 = std::chrono::high_resolution_clock::now();
    lastStepMs_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++stepCount_;
    return lastStepMs_;
}

void PhysicsWorld::stepN(int n) {
    for (int i = 0; i < n; ++i) step();
}

}  // namespace pbx
