#include "Scenarios.h"

#include "PhysicsWorld.h"

#include <cmath>
#include <random>
#include <stdexcept>

namespace pbx {
namespace {

const btScalar kPi = btScalar(3.14159265358979323846);

// Spawn an infinite ground plane at y=0.
int spawnGround(PhysicsWorld& w, btScalar friction = btScalar(0.8),
                btScalar restitution = btScalar(0.0)) {
    BodyDesc g;
    g.name        = "ground";
    g.type        = ShapeType::Ground;
    g.mass        = 0;
    g.friction    = friction;
    g.restitution = restitution;
    return w.spawn(g);
}

// --- stack: N boxes; tests stacking stability under the solver ---------------
ScenarioInfo buildStack(PhysicsWorld& w, const ScenarioParams& p) {
    const int n = p.count > 0 ? p.count : 6;
    spawnGround(w, 0.9f, 0.0f);

    std::mt19937 rng(p.seed);
    std::uniform_real_distribution<btScalar> jitter(-0.002f, 0.002f);

    ScenarioInfo info;
    info.name = "stack";
    info.description = "Vertical stack of boxes — solver stability test.";
    int topId = -1;
    for (int i = 0; i < n; ++i) {
        BodyDesc b;
        b.name        = "box" + std::to_string(i);
        b.type        = ShapeType::Box;
        b.halfExtents = btVector3(0.5f, 0.5f, 0.5f);
        b.mass        = 1.0f;
        b.friction    = 0.9f;
        b.restitution = 0.0f;
        b.position    = btVector3(jitter(rng), 0.5f + i * 1.0f, jitter(rng));
        topId = w.spawn(b);
    }
    info.tracked = {topId};
    // After settling, the top box should rest near this height.
    info.expected = {{"expected_top_y", 0.5 + (n - 1) * 1.0},
                     {"num_boxes", double(n)}};
    return info;
}

// --- bounce: restitution validation -----------------------------------------
ScenarioInfo buildBounce(PhysicsWorld& w, const ScenarioParams& p) {
    const btScalar h = p.dropHeight > 0 ? p.dropHeight : btScalar(5.0);
    const btScalar e = 0.8f;  // ball restitution; ground=1.0 => combined=0.8
    spawnGround(w, 0.5f, 1.0f);

    BodyDesc ball;
    ball.name        = "ball";
    ball.type        = ShapeType::Sphere;
    ball.radius      = 0.5f;
    ball.mass        = 1.0f;
    ball.friction    = 0.2f;
    ball.restitution = e;
    ball.position    = btVector3(0, h, 0);
    const int id = w.spawn(ball);

    ScenarioInfo info;
    info.name = "bounce";
    info.description = "Sphere dropped onto a floor — restitution test.";
    info.tracked = {id};
    // Peak height after first bounce ~ e^2 * (drop distance) + rest height.
    const double dropDist = double(h) - 0.5;   // travel to first contact
    info.expected = {{"drop_height", double(h)},
                     {"restitution", double(e)},
                     {"expected_first_peak_y", e * e * dropDist + 0.5}};
    return info;
}

// --- projectile: pure ballistic motion vs analytic ---------------------------
ScenarioInfo buildProjectile(PhysicsWorld& w, const ScenarioParams& p) {
    // No ground: keep it a clean parabola for validation.
    const btScalar y0 = p.dropHeight > 0 ? p.dropHeight : btScalar(10.0);
    const btVector3 v0(5.0f, 8.0f, 0.0f);

    BodyDesc s;
    s.name           = "projectile";
    s.type           = ShapeType::Sphere;
    s.radius         = 0.2f;
    s.mass           = 1.0f;
    s.linearDamping  = 0.0f;   // no drag -> matches analytic parabola
    s.angularDamping = 0.0f;
    s.position       = btVector3(0, y0, 0);
    s.linearVelocity = v0;
    const int id = w.spawn(s);

    ScenarioInfo info;
    info.name = "projectile";
    info.description = "Ballistic sphere — trajectory vs analytic parabola.";
    info.tracked = {id};
    const double g = w.config().gravity.length();
    info.expected = {{"x0", 0.0}, {"y0", double(y0)},
                     {"vx0", double(v0.x())}, {"vy0", double(v0.y())},
                     {"g", g},
                     {"apex_time", double(v0.y()) / g},
                     {"apex_y", double(y0) + double(v0.y()) * double(v0.y()) / (2 * g)}};
    return info;
}

// --- pendulum: point-to-point joint + energy conservation --------------------
ScenarioInfo buildPendulum(PhysicsWorld& w, const ScenarioParams& p) {
    (void)p;
    const btScalar L = 2.0f;                 // rod length
    const btVector3 pivot(0, 5, 0);          // world anchor
    const btVector3 bobPos = pivot + btVector3(L, 0, 0);  // released horizontally

    BodyDesc bob;
    bob.name        = "bob";
    bob.type        = ShapeType::Sphere;
    bob.radius      = 0.25f;
    bob.mass        = 1.0f;
    bob.friction    = 0.0f;
    bob.linearDamping  = 0.0f;
    bob.angularDamping = 0.0f;
    bob.position    = bobPos;
    const int id = w.spawn(bob);

    // Pin the bob's local point that currently sits at `pivot` to the world.
    // Local pivot = pivot - bobPos = (-L, 0, 0).
    w.addPoint2Point(id, btVector3(-L, 0, 0), /*world*/ -1, pivot);

    ScenarioInfo info;
    info.name = "pendulum";
    info.description = "Point-to-point joint pendulum — energy conservation.";
    info.tracked = {id};
    const double g = w.config().gravity.length();
    // Initial energy (bob at pivot height, at rest): E = m*g*y.
    info.expected = {{"rod_length", double(L)},
                     {"g", g},
                     {"initial_height", double(bobPos.y())}};
    return info;
}

// --- friction ramp: box on an incline ----------------------------------------
ScenarioInfo buildFrictionRamp(PhysicsWorld& w, const ScenarioParams& p) {
    const btScalar deg = p.angleDeg > 0 ? p.angleDeg : btScalar(25.0);
    const btScalar theta = deg * kPi / 180.0f;
    const btScalar boxFriction = p.friction >= 0 ? p.friction : btScalar(0.8);

    // Incline: thin static box rotated about Z by theta.
    btQuaternion rot(btVector3(0, 0, 1), theta);
    BodyDesc ramp;
    ramp.name        = "ramp";
    ramp.type        = ShapeType::Box;
    ramp.halfExtents = btVector3(6.0f, 0.25f, 6.0f);
    ramp.mass        = 0;
    ramp.friction    = 1.0f;   // effective mu = 1.0 * boxFriction
    ramp.orientation = rot;
    ramp.position    = btVector3(0, 0, 0);
    w.spawn(ramp);

    // Surface normal after rotation, and a resting box on it.
    const btVector3 n(-std::sin(theta), std::cos(theta), 0.0f);
    const btScalar offset = 0.25f + 0.5f + 0.01f;
    BodyDesc box;
    box.name        = "slider";
    box.type        = ShapeType::Box;
    box.halfExtents = btVector3(0.5f, 0.5f, 0.5f);
    box.mass        = 1.0f;
    box.friction    = boxFriction;
    box.restitution = 0.0f;
    box.orientation = rot;
    box.position    = n * offset;
    const int id = w.spawn(box);

    ScenarioInfo info;
    info.name = "friction_ramp";
    info.description = "Box on an incline — static vs kinetic friction.";
    info.tracked = {id};
    const double tanTheta = std::tan(theta);
    info.expected = {{"angle_deg", double(deg)},
                     {"mu", double(boxFriction)},
                     {"tan_theta", tanTheta},
                     {"predicted_static", boxFriction >= tanTheta ? 1.0 : 0.0}};
    return info;
}

// --- domino: contact cascade -------------------------------------------------
ScenarioInfo buildDomino(PhysicsWorld& w, const ScenarioParams& p) {
    const int n = p.count > 0 ? p.count : 8;
    spawnGround(w, 0.8f, 0.0f);

    const btScalar spacing = 0.9f;
    int lastId = -1;
    for (int i = 0; i < n; ++i) {
        BodyDesc d;
        d.name        = "domino" + std::to_string(i);
        d.type        = ShapeType::Box;
        d.halfExtents = btVector3(0.1f, 0.75f, 0.4f);  // thin & tall
        d.mass        = 1.0f;
        d.friction    = 0.7f;
        d.restitution = 0.0f;
        d.position    = btVector3(i * spacing, 0.75f, 0);
        lastId = w.spawn(d);
    }
    // Topple the first domino toward +x by giving it a nudge.
    if (btRigidBody* first = w.body(1)) {  // id 0 is the ground
        first->setLinearVelocity(btVector3(2.0f, 0, 0));
        first->setAngularVelocity(btVector3(0, 0, -3.0f));
        first->activate(true);
    }

    ScenarioInfo info;
    info.name = "domino";
    info.description = "Row of dominoes — collision-driven chain reaction.";
    info.tracked = {lastId};
    info.expected = {{"num_dominoes", double(n)}};
    return info;
}

// --- ccd: tunnelling test ----------------------------------------------------
ScenarioInfo buildCcd(PhysicsWorld& w, const ScenarioParams& p) {
    const btScalar wallX = 5.0f;

    // Thin static wall the fast body could tunnel through without CCD.
    BodyDesc wall;
    wall.name        = "wall";
    wall.type        = ShapeType::Box;
    wall.halfExtents = btVector3(0.02f, 2.0f, 2.0f);  // very thin in X
    wall.mass        = 0;
    wall.friction    = 0.5f;
    wall.position    = btVector3(wallX, 0, 0);
    w.spawn(wall);

    // Small, fast projectile aimed at the wall.
    BodyDesc bullet;
    bullet.name           = "bullet";
    bullet.type           = ShapeType::Sphere;
    bullet.radius         = 0.1f;
    bullet.mass           = 1.0f;
    bullet.friction       = 0.0f;
    bullet.restitution    = 0.0f;
    bullet.linearDamping  = 0.0f;
    bullet.position       = btVector3(-5.0f, 0, 0);
    bullet.linearVelocity = btVector3(150.0f, 0, 0);  // very fast
    bullet.ccd            = p.ccd;
    const int id = w.spawn(bullet);
    // Gravity off-axis irrelevant here; keep default gravity.

    ScenarioInfo info;
    info.name = "ccd";
    info.description = "Fast sphere vs thin wall — continuous collision detection.";
    info.tracked = {id};
    info.expected = {{"wall_x", double(wallX)},
                     {"ccd_enabled", p.ccd ? 1.0 : 0.0}};
    return info;
}

// --- raycast: fixed layout for picking / line-of-sight demos ------------------
ScenarioInfo buildRaycast(PhysicsWorld& w, const ScenarioParams& p) {
    (void)p;
    spawnGround(w, 0.8f, 0.0f);

    const btScalar xs[3] = {-3.0f, 0.0f, 3.0f};
    int ids[3];
    for (int i = 0; i < 3; ++i) {
        BodyDesc b;
        b.name        = "target" + std::to_string(i);
        b.type        = ShapeType::Box;
        b.halfExtents = btVector3(0.5f, 0.5f, 0.5f);
        b.mass        = 0;  // static so positions are exact for the demo
        b.position    = btVector3(xs[i], 1.0f, 0);
        ids[i]        = w.spawn(b);
    }

    ScenarioInfo info;
    info.name = "raycast";
    info.description = "Three boxes for object-picking and line-of-sight queries.";
    info.tracked = {ids[0], ids[1], ids[2]};
    info.expected = {{"box_left_x", -3.0}, {"box_mid_x", 0.0}, {"box_right_x", 3.0}};
    return info;
}

// --- perf: stress test for benchmarking --------------------------------------
ScenarioInfo buildPerf(PhysicsWorld& w, const ScenarioParams& p) {
    const int n = p.count > 0 ? p.count : 200;
    spawnGround(w, 0.6f, 0.1f);

    std::mt19937 rng(p.seed);
    std::uniform_real_distribution<btScalar> xz(-5.0f, 5.0f);
    std::uniform_real_distribution<btScalar> yy(2.0f, 20.0f);

    for (int i = 0; i < n; ++i) {
        BodyDesc b;
        b.name        = "b" + std::to_string(i);
        b.type        = (i % 2) ? ShapeType::Sphere : ShapeType::Box;
        b.radius      = 0.3f;
        b.halfExtents = btVector3(0.3f, 0.3f, 0.3f);
        b.mass        = 1.0f;
        b.friction    = 0.5f;
        b.restitution = 0.1f;
        b.position    = btVector3(xz(rng), yy(rng), xz(rng));
        w.spawn(b);
    }

    ScenarioInfo info;
    info.name = "perf";
    info.description = "Many falling bodies — performance benchmark.";
    info.expected = {{"num_bodies", double(n)}};
    return info;
}

}  // namespace

ScenarioInfo buildScenario(const std::string& name, PhysicsWorld& w,
                           const ScenarioParams& p) {
    if (name == "stack")         return buildStack(w, p);
    if (name == "bounce")        return buildBounce(w, p);
    if (name == "projectile")    return buildProjectile(w, p);
    if (name == "pendulum")      return buildPendulum(w, p);
    if (name == "friction_ramp") return buildFrictionRamp(w, p);
    if (name == "domino")        return buildDomino(w, p);
    if (name == "ccd")           return buildCcd(w, p);
    if (name == "raycast")       return buildRaycast(w, p);
    if (name == "perf")          return buildPerf(w, p);
    throw std::runtime_error("unknown scenario: " + name);
}

std::vector<std::pair<std::string, std::string>> listScenarios() {
    return {
        {"stack",         "Vertical stack of boxes — solver stability."},
        {"bounce",        "Sphere drop — restitution / bounce height."},
        {"projectile",    "Ballistic sphere — trajectory vs analytic."},
        {"pendulum",      "Point-to-point joint — energy conservation."},
        {"friction_ramp", "Box on an incline — friction thresholds."},
        {"domino",        "Domino chain — collision cascade."},
        {"ccd",           "Fast sphere vs thin wall — CCD tunnelling."},
        {"raycast",       "Object picking + line-of-sight queries."},
        {"perf",          "Many bodies — performance benchmark."},
    };
}

}  // namespace pbx
