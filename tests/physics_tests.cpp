// physics_tests.cpp — native assertions on the engine API.
//
// These exercise the core behaviours directly (no CSV round-trip) so a broken
// build fails fast and loudly. Run via `ctest` or the `sandbox_tests` binary.
#include "Common.h"
#include "PhysicsWorld.h"
#include "RayCaster.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

using namespace pbx;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& msg) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failures;
}

bool approx(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// A dropped sphere comes to rest at exactly its radius above the ground.
void testRestingHeight() {
    std::puts("testRestingHeight");
    PhysicsWorld w;
    BodyDesc ground; ground.type = ShapeType::Ground; ground.mass = 0;
    w.spawn(ground);
    BodyDesc ball; ball.type = ShapeType::Sphere; ball.radius = 0.5f;
    ball.mass = 1.0f; ball.position = btVector3(0, 8, 0);
    int id = w.spawn(ball);
    w.stepN(400);
    double y = w.body(id)->getWorldTransform().getOrigin().y();
    check(approx(y, 0.5, 0.02), "sphere rests at y=radius (got " + std::to_string(y) + ")");
}

// Free fall for T seconds matches y0 - 0.5*g*T^2 closely.
void testFreeFall() {
    std::puts("testFreeFall");
    WorldConfig cfg; cfg.timeStep = 1.0 / 240.0; cfg.subSteps = 1;
    PhysicsWorld w(cfg);
    BodyDesc s; s.type = ShapeType::Sphere; s.radius = 0.1f; s.mass = 1.0f;
    s.position = btVector3(0, 100, 0);
    int id = w.spawn(s);
    const int n = 240;  // 1 second
    w.stepN(n);
    double t = n * cfg.timeStep;
    double expected = 100.0 - 0.5 * 9.81 * t * t;
    double y = w.body(id)->getWorldTransform().getOrigin().y();
    check(approx(y, expected, 0.1),
          "free fall y after 1s ~= " + std::to_string(expected) +
          " (got " + std::to_string(y) + ")");
}

// A bouncy ball rebounds to roughly e^2 of its drop distance.
void testRestitution() {
    std::puts("testRestitution");
    PhysicsWorld w;
    BodyDesc ground; ground.type = ShapeType::Ground; ground.mass = 0;
    ground.restitution = 1.0f; w.spawn(ground);
    BodyDesc ball; ball.type = ShapeType::Sphere; ball.radius = 0.5f;
    ball.mass = 1.0f; ball.restitution = 0.8f; ball.position = btVector3(0, 5.0f, 0);
    int id = w.spawn(ball);

    double peak = 0.0;
    bool contacted = false;
    for (int i = 0; i < 400; ++i) {
        w.step();
        double y = w.body(id)->getWorldTransform().getOrigin().y();
        if (!contacted && y <= 0.55) contacted = true;
        if (contacted) peak = std::max(peak, y);
    }
    // Drop distance 4.5, e^2 = 0.64 -> peak height ~ 0.64*4.5 + 0.5 ~ 3.38.
    double expected = 0.64 * 4.5 + 0.5;
    check(peak > 2.0 && peak < expected + 0.6,
          "first-bounce peak in expected band (got " + std::to_string(peak) + ")");
}

// Ray picking recovers the correct body id and hit point.
void testRaycastPick() {
    std::puts("testRaycastPick");
    PhysicsWorld w;
    BodyDesc box; box.type = ShapeType::Box; box.halfExtents = btVector3(1, 1, 1);
    box.mass = 0; box.position = btVector3(0, 0, 0);
    int id = w.spawn(box);
    RayCaster rc(w);
    RayHit h = rc.closest(btVector3(0, 10, 0), btVector3(0, -10, 0));
    check(h.hit, "ray hits the box");
    check(h.bodyId == id, "ray reports correct body id");
    check(approx(h.point.y(), 1.0, 0.01),
          "hit point at top face y=1 (got " + std::to_string(h.point.y()) + ")");
}

// Line-of-sight is blocked by an interposed body and clear otherwise.
void testLineOfSight() {
    std::puts("testLineOfSight");
    PhysicsWorld w;
    BodyDesc wall; wall.type = ShapeType::Box; wall.halfExtents = btVector3(0.2f, 2, 2);
    wall.mass = 0; wall.position = btVector3(0, 0, 0);
    w.spawn(wall);
    RayCaster rc(w);
    check(!rc.lineOfSight(btVector3(-5, 0, 0), btVector3(5, 0, 0)),
          "LOS blocked through the wall");
    check(rc.lineOfSight(btVector3(-5, 5, 0), btVector3(5, 5, 0)),
          "LOS clear above the wall");
}

// A point-to-point joint keeps the bob at a fixed distance from the pivot.
void testJointConstraint() {
    std::puts("testJointConstraint");
    // More solver iterations => a stiffer joint. Sequential-impulse joints are
    // soft; ~40 iterations holds the rod length to well under 1%.
    WorldConfig cfg; cfg.solverIterations = 40;
    PhysicsWorld w(cfg);
    const btVector3 pivot(0, 5, 0);
    const btScalar L = 2.0f;
    BodyDesc bob; bob.type = ShapeType::Sphere; bob.radius = 0.2f; bob.mass = 1.0f;
    bob.position = pivot + btVector3(L, 0, 0);
    int id = w.spawn(bob);
    w.addPoint2Point(id, btVector3(-L, 0, 0), -1, pivot);

    double maxErr = 0.0;
    for (int i = 0; i < 600; ++i) {
        w.step();
        btVector3 p = w.body(id)->getWorldTransform().getOrigin();
        double dist = (p - pivot).length();
        maxErr = std::max(maxErr, std::fabs(dist - double(L)));
    }
    check(maxErr < 0.05,
          "bob stays ~L from pivot (max err " + std::to_string(maxErr) + ")");
}

// A stack of boxes stays upright (top box near expected height, small drift).
void testStackStability() {
    std::puts("testStackStability");
    PhysicsWorld w;
    BodyDesc ground; ground.type = ShapeType::Ground; ground.mass = 0;
    ground.friction = 0.9f; w.spawn(ground);
    int top = -1;
    for (int i = 0; i < 5; ++i) {
        BodyDesc b; b.type = ShapeType::Box; b.halfExtents = btVector3(0.5f, 0.5f, 0.5f);
        b.mass = 1.0f; b.friction = 0.9f; b.position = btVector3(0, 0.5f + i, 0);
        top = w.spawn(b);
    }
    w.stepN(400);
    btVector3 p = w.body(top)->getWorldTransform().getOrigin();
    check(approx(p.y(), 4.5, 0.15),
          "top box near y=4.5 (got " + std::to_string(p.y()) + ")");
    check(std::fabs(p.x()) < 0.3 && std::fabs(p.z()) < 0.3,
          "stack did not topple (drift small)");
}

// CCD stops a fast body that would otherwise tunnel through a thin wall.
void testCcd() {
    std::puts("testCcd");
    auto runsThrough = [](bool ccd) {
        PhysicsWorld w;
        w.setGravity(btVector3(0, 0, 0));  // isolate the horizontal motion
        BodyDesc wall; wall.type = ShapeType::Box; wall.halfExtents = btVector3(0.02f, 2, 2);
        wall.mass = 0; wall.position = btVector3(5, 0, 0); w.spawn(wall);
        BodyDesc b; b.type = ShapeType::Sphere; b.radius = 0.1f; b.mass = 1.0f;
        b.position = btVector3(-5, 0, 0); b.linearVelocity = btVector3(150, 0, 0);
        b.ccd = ccd;
        int id = w.spawn(b);
        w.stepN(60);
        return w.body(id)->getWorldTransform().getOrigin().x() > 5.5;  // passed wall?
    };
    check(!runsThrough(true),  "with CCD the fast sphere is stopped by the wall");
    check(runsThrough(false),  "without CCD the fast sphere tunnels through");
}

}  // namespace

int main() {
    std::puts("=== physics-sandbox native tests ===");
    testRestingHeight();
    testFreeFall();
    testRestitution();
    testRaycastPick();
    testLineOfSight();
    testJointConstraint();
    testStackStability();
    testCcd();
    std::printf("\n%s (%d failure%s)\n",
                g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
