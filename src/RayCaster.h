// RayCaster.h — ray-casting against the collision world.
//
// Two jobs, both run against the live world without pausing the simulation:
//   * object picking  — which body sits under a ray (e.g. the mouse cursor)?
//   * line-of-sight   — is the straight segment between two points unobstructed?
#pragma once

#include "Common.h"

#include <vector>

namespace pbx {

class PhysicsWorld;

class RayCaster {
public:
    explicit RayCaster(PhysicsWorld& world) : world_(world) {}

    // Closest hit along the segment [from, to].
    RayHit closest(const btVector3& from, const btVector3& to) const;

    // Pick the body under a ray defined by origin + direction, up to maxDist.
    // Returns the hit (hit=false if nothing within range). Bodies in `ignore`
    // are skipped (handy for ignoring the camera's own body).
    RayHit pick(const btVector3& origin, const btVector3& direction,
                btScalar maxDist,
                const std::vector<int>& ignore = {}) const;

    // True if nothing blocks the segment between `from` and `to`. Endpoints'
    // own bodies are typically passed in `ignore` so they don't self-occlude.
    bool lineOfSight(const btVector3& from, const btVector3& to,
                     const std::vector<int>& ignore = {}) const;

private:
    PhysicsWorld& world_;
};

}  // namespace pbx
