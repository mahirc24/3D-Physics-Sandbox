#include "DebugDrawer.h"

#include "PhysicsWorld.h"

#include <fstream>

namespace pbx {

void DebugDrawer::capture(PhysicsWorld& world) {
    clear();
    btIDebugDraw* previous = world.world()->getDebugDrawer();
    world.world()->setDebugDrawer(this);
    world.world()->debugDrawWorld();       // triggers drawLine / drawContactPoint
    world.world()->setDebugDrawer(previous);
}

void DebugDrawer::exportTo(const std::string& path) const {
    std::ofstream out(path);
    // A trivially-parseable text format:
    //   L x0 y0 z0 x1 y1 z1 r g b    (wireframe / constraint line)
    //   C px py pz nx ny nz dist      (contact point + normal + penetration)
    out << "# physics-sandbox debug geometry\n";
    out << "# lines=" << lines_.size() << " contacts=" << contacts_.size() << "\n";
    for (const auto& l : lines_) {
        out << "L " << l.from.x() << ' ' << l.from.y() << ' ' << l.from.z() << ' '
            << l.to.x()   << ' ' << l.to.y()   << ' ' << l.to.z()   << ' '
            << l.color.x() << ' ' << l.color.y() << ' ' << l.color.z() << "\n";
    }
    for (const auto& c : contacts_) {
        out << "C " << c.point.x() << ' ' << c.point.y() << ' ' << c.point.z() << ' '
            << c.normal.x() << ' ' << c.normal.y() << ' ' << c.normal.z() << ' '
            << c.distance << "\n";
    }
}

}  // namespace pbx
