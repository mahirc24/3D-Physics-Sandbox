#include "RayCaster.h"

#include "PhysicsWorld.h"

#include <algorithm>

namespace pbx {

RayHit RayCaster::closest(const btVector3& from, const btVector3& to) const {
    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    world_.world()->rayTest(from, to, cb);

    RayHit r;
    if (cb.hasHit()) {
        r.hit      = true;
        r.point    = cb.m_hitPointWorld;
        r.normal   = cb.m_hitNormalWorld;
        r.fraction = cb.m_closestHitFraction;
        r.distance = (r.point - from).length();
        const btCollisionObject* obj = cb.m_collisionObject;
        r.bodyId = obj ? obj->getUserIndex() : -1;
    }
    return r;
}

RayHit RayCaster::pick(const btVector3& origin, const btVector3& direction,
                       btScalar maxDist,
                       const std::vector<int>& ignore) const {
    const btVector3 dir = direction.normalized();
    const btVector3 to  = origin + dir * maxDist;

    // AllHits so we can skip ignored bodies and keep the nearest remaining one.
    btCollisionWorld::AllHitsRayResultCallback cb(origin, to);
    world_.world()->rayTest(origin, to, cb);

    RayHit best;
    btScalar bestFrac = btScalar(2);  // sentinel > any real fraction
    for (int i = 0; i < cb.m_collisionObjects.size(); ++i) {
        const btCollisionObject* obj = cb.m_collisionObjects[i];
        const int id = obj ? obj->getUserIndex() : -1;
        if (std::find(ignore.begin(), ignore.end(), id) != ignore.end()) continue;

        const btScalar frac = cb.m_hitFractions[i];
        if (frac < bestFrac) {
            bestFrac      = frac;
            best.hit      = true;
            best.bodyId   = id;
            best.point    = cb.m_hitPointWorld[i];
            best.normal   = cb.m_hitNormalWorld[i];
            best.fraction = frac;
            best.distance = (best.point - origin).length();
        }
    }
    return best;
}

bool RayCaster::lineOfSight(const btVector3& from, const btVector3& to,
                            const std::vector<int>& ignore) const {
    btCollisionWorld::AllHitsRayResultCallback cb(from, to);
    world_.world()->rayTest(from, to, cb);

    for (int i = 0; i < cb.m_collisionObjects.size(); ++i) {
        const btCollisionObject* obj = cb.m_collisionObjects[i];
        const int id = obj ? obj->getUserIndex() : -1;
        if (std::find(ignore.begin(), ignore.end(), id) != ignore.end()) continue;
        // Any un-ignored intersection before the endpoint blocks the view.
        return false;
    }
    return true;
}

}  // namespace pbx
