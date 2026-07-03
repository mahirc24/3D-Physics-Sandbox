#include "RigidBodyFactory.h"

namespace pbx {
namespace RigidBodyFactory {

btCollisionShape* createShape(const BodyDesc& d) {
    switch (d.type) {
        case ShapeType::Box:
            return new btBoxShape(d.halfExtents);
        case ShapeType::Sphere:
            return new btSphereShape(d.radius);
        case ShapeType::Capsule:
            // btCapsuleShape(radius, height) — height is the cylinder part only.
            return new btCapsuleShape(d.radius, d.height);
        case ShapeType::Cylinder:
            // btCylinderShape takes half-extents; Y is the axis.
            return new btCylinderShape(
                btVector3(d.radius, d.height * btScalar(0.5), d.radius));
        case ShapeType::Plane:
            return new btStaticPlaneShape(d.planeNormal.normalized(),
                                          d.planeConstant);
        case ShapeType::Ground:
        default:
            return new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    }
}

std::unique_ptr<BodyRecord> create(const BodyDesc& d, int id) {
    auto rec  = std::make_unique<BodyRecord>();
    rec->id   = id;
    rec->name = d.name;
    rec->mass = d.mass;

    rec->shape.reset(createShape(d));

    // Static bodies (mass 0) have zero local inertia by convention.
    const bool isDynamic = d.mass != btScalar(0);
    btVector3 inertia(0, 0, 0);
    if (isDynamic) rec->shape->calculateLocalInertia(d.mass, inertia);

    btTransform start;
    start.setIdentity();
    start.setOrigin(d.position);
    start.setRotation(d.orientation);

    rec->motion.reset(new btDefaultMotionState(start));

    btRigidBody::btRigidBodyConstructionInfo ci(
        d.mass, rec->motion.get(), rec->shape.get(), inertia);
    ci.m_friction        = d.friction;
    ci.m_restitution     = d.restitution;
    ci.m_rollingFriction = d.rollingFriction;
    ci.m_spinningFriction = d.spinningFriction;
    ci.m_linearDamping   = d.linearDamping;
    ci.m_angularDamping  = d.angularDamping;

    rec->body.reset(new btRigidBody(ci));

    // Initial velocities.
    if (isDynamic) {
        rec->body->setLinearVelocity(d.linearVelocity);
        rec->body->setAngularVelocity(d.angularVelocity);
    }

    // Continuous collision detection for thin/fast objects.
    if (d.ccd) {
        btScalar motionThresh = d.ccdMotionThreshold;
        btScalar sweptRadius  = d.ccdSweptSphereRadius;
        if (motionThresh <= 0) motionThresh = d.radius > 0 ? d.radius : btScalar(0.05);
        if (sweptRadius  <= 0) sweptRadius  = (d.radius > 0 ? d.radius : btScalar(0.1)) * btScalar(0.5);
        rec->body->setCcdMotionThreshold(motionThresh);
        rec->body->setCcdSweptSphereRadius(sweptRadius);
    }

    // Stash the id so ray hits / contact reports can identify the body.
    rec->body->setUserIndex(id);

    return rec;
}

}  // namespace RigidBodyFactory
}  // namespace pbx
