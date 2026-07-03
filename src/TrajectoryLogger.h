// TrajectoryLogger.h — records the motion of every dynamic body over time.
//
// Each recorded frame captures position, orientation, and velocity so the Python
// tools can plot trajectories, measure bounce heights, check energy, and diff
// against a golden run in regression tests.
#pragma once

#include "Common.h"

#include <string>
#include <vector>

namespace pbx {

class PhysicsWorld;

class TrajectoryLogger {
public:
    struct Sample {
        int      step;
        double   time;
        int      bodyId;
        btVector3 pos;
        btQuaternion rot;
        btVector3 linVel;
        btVector3 angVel;
        bool     active;
    };

    // Record the current state of all dynamic bodies for this step/time.
    void record(const PhysicsWorld& world);

    // Write all samples to CSV (one row per body per recorded step).
    void writeCsv(const std::string& path) const;

    const std::vector<Sample>& samples() const { return samples_; }
    void clear() { samples_.clear(); }

private:
    std::vector<Sample> samples_;
};

}  // namespace pbx
