// Metrics.h — simulation metrics and step-timing statistics.
//
// Collects the numbers the Python harness validates against expected physical
// behavior (energy, resting state, contact counts) and the timing stats used by
// the performance-analysis script. Serialises to JSON via JsonWriter.
#pragma once

#include "Common.h"

#include <string>
#include <vector>

namespace pbx {

class PhysicsWorld;

// Rolling timing statistics fed one step duration at a time.
class StepTimer {
public:
    void add(double ms);
    double total() const { return total_; }
    double mean()  const { return count_ ? total_ / count_ : 0.0; }
    double min()   const { return count_ ? min_ : 0.0; }
    double max()   const { return max_; }
    int    count() const { return count_; }
    // Steps-per-second based on mean step time.
    double stepsPerSecond() const;

private:
    double total_ = 0.0;
    double min_   = 1e300;
    double max_   = 0.0;
    int    count_ = 0;
};

// Instantaneous snapshot of the world.
struct WorldSnapshot {
    double kineticEnergy   = 0.0;
    double potentialEnergy = 0.0;  // relative to y=0, using |gravity|
    double totalEnergy     = 0.0;
    int    manifolds       = 0;    // narrow-phase contact manifolds
    int    contactPoints   = 0;
    int    activeBodies    = 0;
    int    totalBodies     = 0;
};

WorldSnapshot snapshot(PhysicsWorld& world);

// Serialise a snapshot + timing + a small named-scalar bag into JSON.
std::string metricsJson(const WorldSnapshot& snap, const StepTimer& timer,
                        const std::vector<std::pair<std::string, double>>& extras);

}  // namespace pbx
