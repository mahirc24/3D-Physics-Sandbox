// Scenarios.h — data-driven demo scenes built on the modular framework.
//
// Each scenario is a builder that populates a PhysicsWorld with bodies and
// constraints and returns analytic expectations the Python harness validates
// against. Together they exercise every capability: contacts, gravity, friction,
// restitution, joints, stacking stability, CCD, and ray queries.
#pragma once

#include "Common.h"

#include <string>
#include <utility>
#include <vector>

namespace pbx {

class PhysicsWorld;

struct ScenarioParams {
    int      count      = 0;    // 0  => use scenario default (stack height / #bodies)
    btScalar dropHeight = 0;    // 0  => default (drop / launch height)
    btScalar angleDeg   = 0;    // 0  => default (ramp angle)
    btScalar friction   = -1;   // <0 => default
    bool     ccd        = true; // CCD on/off (ccd scenario)
    unsigned seed       = 1;
};

struct ScenarioInfo {
    std::string name;
    std::string description;
    // Analytic expectations / notes, folded into metrics.json for validation.
    std::vector<std::pair<std::string, double>> expected;
    // Body ids the analysis should focus on (empty => all dynamic bodies).
    std::vector<int> tracked;
};

// Build `name` into `world`. Unknown names throw std::runtime_error.
ScenarioInfo buildScenario(const std::string& name, PhysicsWorld& world,
                           const ScenarioParams& p);

// (name, description) for every registered scenario.
std::vector<std::pair<std::string, std::string>> listScenarios();

}  // namespace pbx
