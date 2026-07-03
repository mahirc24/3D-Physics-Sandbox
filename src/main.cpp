// main.cpp — command-line driver for the physics sandbox.
//
// Builds a world, constructs a named scenario, steps it forward while logging
// trajectories and per-step timing, then writes a trajectory CSV, a metrics
// JSON, and (optionally) captured debug geometry. The Python harness drives this
// binary across scenarios to automate testing and performance analysis.
#include "DebugDrawer.h"
#include "Metrics.h"
#include "PhysicsWorld.h"
#include "RayCaster.h"
#include "Scenarios.h"
#include "TrajectoryLogger.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

using namespace pbx;

namespace {

struct Args {
    std::string scenario = "stack";
    int    steps    = 300;
    double timeStep = 1.0 / 60.0;
    double fixedStep = 1.0 / 120.0;
    int    subSteps = 10;
    double gravity  = 9.81;   // magnitude, applied as -y
    ScenarioParams sp;
    std::string outCsv, outMetrics, outDebug;
    bool quiet = false;
    bool list  = false;
};

void printUsage() {
    std::cout <<
        "physics-sandbox — 3D rigid-body sandbox (Bullet)\n\n"
        "Usage: sandbox [options]\n"
        "  --scenario NAME    scenario to run (default: stack)\n"
        "  --steps N          number of simulation steps (default: 300)\n"
        "  --timestep DT      seconds per step (default: 0.016667)\n"
        "  --fixedstep FS     internal fixed step (default: 0.008333 = 1/120)\n"
        "  --substeps K       max solver substeps per step (default: 10)\n"
        "  --gravity G        gravity magnitude, applied as -Y (default: 9.81)\n"
        "  --count N          scenario body/stack count\n"
        "  --drop H           drop/launch height\n"
        "  --angle D          ramp angle in degrees\n"
        "  --friction F       object friction\n"
        "  --no-ccd           disable continuous collision detection (ccd scene)\n"
        "  --seed S           RNG seed (default: 1)\n"
        "  --out FILE         write trajectory CSV\n"
        "  --metrics FILE     write metrics JSON\n"
        "  --debug FILE       capture + write debug geometry\n"
        "  --quiet            suppress the summary line\n"
        "  --list             list scenarios and exit\n";
}

bool matches(const char* a, const char* b) { return std::strcmp(a, b) == 0; }

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const char* s = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { std::cerr << "missing value for " << s << "\n"; std::exit(2); }
            return argv[++i];
        };
        if      (matches(s, "--scenario")) a.scenario = next();
        else if (matches(s, "--steps"))    a.steps = std::stoi(next());
        else if (matches(s, "--timestep")) a.timeStep = std::stod(next());
        else if (matches(s, "--fixedstep")) a.fixedStep = std::stod(next());
        else if (matches(s, "--substeps")) a.subSteps = std::stoi(next());
        else if (matches(s, "--gravity"))  a.gravity = std::stod(next());
        else if (matches(s, "--count"))    a.sp.count = std::stoi(next());
        else if (matches(s, "--drop"))     a.sp.dropHeight = std::stof(next());
        else if (matches(s, "--angle"))    a.sp.angleDeg = std::stof(next());
        else if (matches(s, "--friction")) a.sp.friction = std::stof(next());
        else if (matches(s, "--no-ccd"))   a.sp.ccd = false;
        else if (matches(s, "--seed"))     a.sp.seed = std::stoul(next());
        else if (matches(s, "--out"))      a.outCsv = next();
        else if (matches(s, "--metrics"))  a.outMetrics = next();
        else if (matches(s, "--debug"))    a.outDebug = next();
        else if (matches(s, "--quiet"))    a.quiet = true;
        else if (matches(s, "--list"))     a.list = true;
        else if (matches(s, "-h") || matches(s, "--help")) { printUsage(); std::exit(0); }
        else { std::cerr << "unknown option: " << s << "\n"; printUsage(); std::exit(2); }
    }
    return a;
}

// Ray-cast picking + line-of-sight demo, printed and folded into metrics.
void raycastDemo(PhysicsWorld& world, std::vector<std::pair<std::string, double>>& extras,
                 bool quiet) {
    RayCaster rc(world);

    // 1) Pick straight down onto the left box (x = -3).
    RayHit down = rc.closest(btVector3(-3, 6, 0), btVector3(-3, 0, 0));
    // 2) Pick the nearest box along -X from far right; nearest is the box at x=3.
    RayHit horiz = rc.pick(btVector3(10, 1, 0), btVector3(-1, 0, 0), 30.0f);
    // 3) LOS through the row of boxes at box height => blocked.
    bool losBlocked = rc.lineOfSight(btVector3(-6, 1, 0), btVector3(6, 1, 0));
    // 4) LOS above the boxes => clear.
    bool losClear = rc.lineOfSight(btVector3(-6, 5, 0), btVector3(6, 5, 0));

    if (!quiet) {
        std::cout << "[raycast] pick-down    hit=" << down.hit
                  << " body=" << down.bodyId
                  << " point=(" << down.point.x() << "," << down.point.y()
                  << "," << down.point.z() << ")\n";
        std::cout << "[raycast] pick-nearest hit=" << horiz.hit
                  << " body=" << horiz.bodyId
                  << " dist=" << horiz.distance << "\n";
        std::cout << "[raycast] LOS through row  = "
                  << (losBlocked ? "CLEAR" : "BLOCKED") << " (expect BLOCKED)\n";
        std::cout << "[raycast] LOS above row    = "
                  << (losClear ? "CLEAR" : "BLOCKED") << " (expect CLEAR)\n";
    }

    extras.emplace_back("raycast_down_body", double(down.bodyId));
    extras.emplace_back("raycast_down_point_x", double(down.point.x()));
    extras.emplace_back("raycast_nearest_body", double(horiz.bodyId));
    extras.emplace_back("raycast_nearest_dist", double(horiz.distance));
    extras.emplace_back("los_through_row_blocked", losBlocked ? 0.0 : 1.0);
    extras.emplace_back("los_above_row_clear", losClear ? 1.0 : 0.0);
}

}  // namespace

int main(int argc, char** argv) {
    Args a = parse(argc, argv);

    if (a.list) {
        std::cout << "Available scenarios:\n";
        for (auto& [name, desc] : listScenarios())
            std::cout << "  " << name << std::string(16 - name.size(), ' ') << desc << "\n";
        return 0;
    }

    WorldConfig cfg;
    cfg.gravity  = btVector3(0, -btScalar(a.gravity), 0);
    cfg.timeStep = btScalar(a.timeStep);
    cfg.fixedStep = btScalar(a.fixedStep);
    cfg.subSteps = a.subSteps;

    PhysicsWorld world(cfg);

    ScenarioInfo info;
    try {
        info = buildScenario(a.scenario, world, a.sp);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    TrajectoryLogger logger;
    StepTimer timer;

    // Record the initial frame, then step.
    logger.record(world);
    for (int i = 0; i < a.steps; ++i) {
        timer.add(world.step());
        logger.record(world);
    }

    // Assemble metrics, including scenario expectations and any query results.
    std::vector<std::pair<std::string, double>> extras = info.expected;
    // Surface which bodies the scenario wants analysed.
    extras.emplace_back("tracked_count", double(info.tracked.size()));
    for (size_t i = 0; i < info.tracked.size(); ++i)
        extras.emplace_back("tracked_" + std::to_string(i), double(info.tracked[i]));
    if (a.scenario == "raycast") raycastDemo(world, extras, a.quiet);

    WorldSnapshot snap = snapshot(world);

    if (!a.outCsv.empty())     logger.writeCsv(a.outCsv);
    if (!a.outMetrics.empty()) {
        std::ofstream(a.outMetrics) << metricsJson(snap, timer, extras) << "\n";
    }
    if (!a.outDebug.empty()) {
        DebugDrawer dd;
        dd.capture(world);
        dd.exportTo(a.outDebug);
    }

    if (!a.quiet) {
        std::cout << "[" << a.scenario << "] steps=" << timer.count()
                  << " mean=" << timer.mean() << "ms"
                  << " total_energy=" << snap.totalEnergy
                  << " contacts=" << snap.contactPoints
                  << " active=" << snap.activeBodies << "/" << snap.totalBodies
                  << " sps=" << timer.stepsPerSecond() << "\n";
    }
    return 0;
}
