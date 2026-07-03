#include "Metrics.h"

#include "JsonWriter.h"
#include "PhysicsWorld.h"

#include <algorithm>

namespace pbx {

void StepTimer::add(double ms) {
    total_ += ms;
    min_ = std::min(min_, ms);
    max_ = std::max(max_, ms);
    ++count_;
}

double StepTimer::stepsPerSecond() const {
    const double m = mean();
    return m > 0.0 ? 1000.0 / m : 0.0;
}

WorldSnapshot snapshot(PhysicsWorld& world) {
    WorldSnapshot s;
    const btVector3 g = world.config().gravity;
    const double gMag = g.length();

    for (const auto& rec : world.bodies()) {
        if (!rec || !rec->body) continue;
        s.totalBodies++;
        if (rec->mass == btScalar(0)) continue;  // static: no KE/PE tracked

        btRigidBody* b = rec->body.get();
        const double m = static_cast<double>(rec->mass);

        // Translational + rotational kinetic energy.
        const btVector3 v = b->getLinearVelocity();
        const btVector3 w = b->getAngularVelocity();
        const double linKE = 0.5 * m * v.length2();

        // Rotational KE ~ 0.5 * wᵀ I w, using the diagonal local inertia.
        const btVector3 invI = b->getInvInertiaDiagLocal();
        double rotKE = 0.0;
        if (invI.x() > 0) rotKE += 0.5 * (w.x() * w.x()) / invI.x();
        if (invI.y() > 0) rotKE += 0.5 * (w.y() * w.y()) / invI.y();
        if (invI.z() > 0) rotKE += 0.5 * (w.z() * w.z()) / invI.z();

        s.kineticEnergy += linKE + rotKE;

        const double height = b->getWorldTransform().getOrigin().y();
        s.potentialEnergy += m * gMag * height;

        if (b->isActive()) s.activeBodies++;
    }
    s.totalEnergy = s.kineticEnergy + s.potentialEnergy;

    // Contact manifolds from the narrow-phase dispatcher.
    btDispatcher* disp = world.world()->getDispatcher();
    const int numManifolds = disp->getNumManifolds();
    s.manifolds = numManifolds;
    for (int i = 0; i < numManifolds; ++i) {
        s.contactPoints += disp->getManifoldByIndexInternal(i)->getNumContacts();
    }
    return s;
}

std::string metricsJson(const WorldSnapshot& snap, const StepTimer& timer,
                        const std::vector<std::pair<std::string, double>>& extras) {
    JsonWriter w;
    w.kv("kinetic_energy", snap.kineticEnergy)
     .kv("potential_energy", snap.potentialEnergy)
     .kv("total_energy", snap.totalEnergy)
     .kv("manifolds", snap.manifolds)
     .kv("contact_points", snap.contactPoints)
     .kv("active_bodies", snap.activeBodies)
     .kv("total_bodies", snap.totalBodies)
     .kv("steps", timer.count())
     .kv("step_ms_mean", timer.mean())
     .kv("step_ms_min", timer.min())
     .kv("step_ms_max", timer.max())
     .kv("step_ms_total", timer.total())
     .kv("steps_per_second", timer.stepsPerSecond());
    for (const auto& kv : extras) w.kv(kv.first, kv.second);
    return w.str();
}

}  // namespace pbx
