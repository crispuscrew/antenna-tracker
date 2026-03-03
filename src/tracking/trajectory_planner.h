#pragma once
#include <vector>
#include <ctime>
#include "tle_tracker.h"

struct PassPoint {
    std::time_t t;      // Unix timestamp
    float       az;     // commanded azimuth, degrees — unwrapped, may exceed [0,360)
    float       el;     // elevation, degrees
};

struct Pass {
    std::time_t aos;    // Unix timestamp
    std::time_t los;    // Unix timestamp
    std::vector<PassPoint> trajectory; // 1-second steps, unwrapped azimuth
};

class TrajectoryPlanner {
public:
    explicit TrajectoryPlanner(const TleTracker& tracker);

    // Plan the next pass occurring after 'now'.
    // Returns a Pass with a fully pre-computed trajectory.
    Pass plan_next(std::time_t now) const;

    // Look up the commanded az/el for timestamp 't' within a pre-computed pass.
    // Selects the point with the closest timestamp (no interpolation needed at 1 s
    // step / 10 Hz control rate).  Returns false if trajectory is empty.
    static bool lookup(const Pass& pass, std::time_t t, float& az, float& el);

private:
    const TleTracker& tracker_;
};
