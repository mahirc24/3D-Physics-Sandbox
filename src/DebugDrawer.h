// DebugDrawer.h — visualization utility.
//
// Implements Bullet's btIDebugDraw. In an interactive build these callbacks feed
// an OpenGL line renderer; here (headless) we capture the geometry into buffers
// and export it so the Python side can plot collision shapes, contact points,
// AABBs and constraint frames offline. This is how solver issues and unstable
// contacts become easy to spot while tuning parameters.
#pragma once

#include <LinearMath/btIDebugDraw.h>

#include <string>
#include <vector>

namespace pbx {

class PhysicsWorld;

class DebugDrawer : public btIDebugDraw {
public:
    struct Line { btVector3 from, to, color; };
    struct Contact { btVector3 point, normal; btScalar distance; };

    DebugDrawer() = default;

    // --- btIDebugDraw interface ---------------------------------------------
    void drawLine(const btVector3& from, const btVector3& to,
                  const btVector3& color) override {
        lines_.push_back({from, to, color});
    }
    void drawContactPoint(const btVector3& pointOnB, const btVector3& normalOnB,
                          btScalar distance, int /*lifeTime*/,
                          const btVector3& color) override {
        contacts_.push_back({pointOnB, normalOnB, distance});
        // Also draw a short stub so it shows up in the wireframe view.
        lines_.push_back({pointOnB, pointOnB + normalOnB * btScalar(0.2), color});
    }
    void reportErrorWarning(const char* warning) override {
        lastWarning_ = warning ? warning : "";
    }
    void draw3dText(const btVector3&, const char*) override {}
    void setDebugMode(int mode) override { mode_ = mode; }
    int  getDebugMode() const override { return mode_; }

    // --- capture / export ----------------------------------------------------
    void clear() { lines_.clear(); contacts_.clear(); }

    // Run debugDrawWorld() against the world, refilling the buffers.
    void capture(PhysicsWorld& world);

    // Write captured geometry to a simple text format the Python plotter reads.
    void exportTo(const std::string& path) const;

    const std::vector<Line>&    lines()    const { return lines_; }
    const std::vector<Contact>& contacts() const { return contacts_; }
    const std::string&          lastWarning() const { return lastWarning_; }

private:
    std::vector<Line>    lines_;
    std::vector<Contact> contacts_;
    std::string          lastWarning_;
    int mode_ = DBG_DrawWireframe | DBG_DrawContactPoints |
                DBG_DrawConstraints | DBG_DrawConstraintLimits;
};

}  // namespace pbx
