#include "TrajectoryLogger.h"

#include "PhysicsWorld.h"

#include <fstream>

namespace pbx {

void TrajectoryLogger::record(const PhysicsWorld& world) {
    const int    step = world.stepCount();
    const double time = step * static_cast<double>(world.config().timeStep);

    for (const auto& rec : world.bodies()) {
        if (!rec || !rec->body) continue;
        if (rec->mass == btScalar(0)) continue;  // skip static bodies

        btRigidBody* b = rec->body.get();
        const btTransform& t = b->getWorldTransform();

        Sample s;
        s.step   = step;
        s.time   = time;
        s.bodyId = rec->id;
        s.pos    = t.getOrigin();
        s.rot    = t.getRotation();
        s.linVel = b->getLinearVelocity();
        s.angVel = b->getAngularVelocity();
        s.active = b->isActive();
        samples_.push_back(s);
    }
}

void TrajectoryLogger::writeCsv(const std::string& path) const {
    std::ofstream out(path);
    out << "step,time,body_id,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,active\n";
    out.precision(9);
    for (const auto& s : samples_) {
        out << s.step << ',' << s.time << ',' << s.bodyId << ','
            << s.pos.x() << ',' << s.pos.y() << ',' << s.pos.z() << ','
            << s.rot.x() << ',' << s.rot.y() << ',' << s.rot.z() << ',' << s.rot.w() << ','
            << s.linVel.x() << ',' << s.linVel.y() << ',' << s.linVel.z() << ','
            << s.angVel.x() << ',' << s.angVel.y() << ',' << s.angVel.z() << ','
            << (s.active ? 1 : 0) << "\n";
    }
}

}  // namespace pbx
