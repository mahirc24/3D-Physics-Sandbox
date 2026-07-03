// RigidBodyFactory.h — the "spawning & configuring" half of the modular framework.
//
// Pure creation: given a BodyDesc, build the collision shape, motion state, and
// rigid body, applying mass, inertia, material and CCD settings. Ownership is
// handed back in a BodyRecord; PhysicsWorld is responsible for lifetime and for
// adding/removing the body from the simulation.
#pragma once

#include "Common.h"

namespace pbx {
namespace RigidBodyFactory {

// Build a collision shape for the given descriptor. Caller owns it.
btCollisionShape* createShape(const BodyDesc& d);

// Build a fully-configured rigid body (shape + motion state + material + CCD)
// and return it wrapped in an owning record. `id` is stamped onto the record and
// stored in the body's user index so ray queries can recover it.
std::unique_ptr<BodyRecord> create(const BodyDesc& d, int id);

}  // namespace RigidBodyFactory
}  // namespace pbx
