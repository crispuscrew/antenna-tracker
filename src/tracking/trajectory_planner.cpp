#include "trajectory_planner.h"
#include <cmath>
#include <limits>
#include <algorithm>

TrajectoryPlanner::TrajectoryPlanner(const TleTracker& tracker)
    : tracker_(tracker)
{}

Pass TrajectoryPlanner::plan_next(std::time_t now) const
{
    Pass pass;
    pass.aos = tracker_.next_aos(now);
    pass.los = tracker_.los_after(pass.aos); // AOS уже известен — не искать его повторно

    if (pass.los <= pass.aos) {
        // Shouldn't happen with a valid TLE, but guard anyway.
        return pass;
    }

    // Pre-compute trajectory at 1-second resolution.
    // The azimuth is "unwrapped" so that the antenna never slews more than 180°
    // in one step and never crosses the 0°/360° discontinuity unexpectedly.

    // Для первой точки выбираем offset так, чтобы попасть в [-250, +250].
    // Для всех последующих — выбираем offset, минимизирующий расстояние от
    // предыдущего командованного азимута (unwrap). Клампить промежуточные
    // точки нельзя — это разрывает непрерывность траектории.
    float prev_az = -1e9f; // sentinel

    for (std::time_t t = pass.aos; t <= pass.los; ++t) {
        auto azel = tracker_.compute(t);
        float raw_az = static_cast<float>(azel.az_deg); // [0, 360)
        float el     = static_cast<float>(azel.el_deg);

        const float offsets[4] = {0.f, 360.f, -360.f, -720.f};
        float best = raw_az;

        if (prev_az < -1e8f) {
            // Первая точка: выбрать offset, при котором az ∈ [-250, +250].
            for (float off : offsets) {
                float candidate = raw_az + off;
                if (candidate >= -250.f && candidate <= 250.f) {
                    best = candidate;
                    break;
                }
            }
        } else {
            // Последующие точки: минимизировать расстояние от prev_az (unwrap).
            float min_dist = std::numeric_limits<float>::max();
            for (float off : offsets) {
                float candidate = raw_az + off;
                float dist = std::fabs(candidate - prev_az);
                if (dist < min_dist) {
                    min_dist = dist;
                    best = candidate;
                }
            }
        }

        prev_az = best;
        pass.trajectory.push_back({t, best, el});
    }

    return pass;
}

bool TrajectoryPlanner::lookup(const Pass& pass, std::time_t t, float& az, float& el)
{
    if (pass.trajectory.empty()) return false;

    // Binary-search for the closest timestamp.
    auto it = std::lower_bound(
        pass.trajectory.begin(), pass.trajectory.end(), t,
        [](const PassPoint& p, std::time_t tv) { return p.t < tv; });

    if (it == pass.trajectory.end())  --it;
    if (it != pass.trajectory.begin()) {
        auto prev = std::prev(it);
        if (std::abs(static_cast<long long>(prev->t - t)) <
            std::abs(static_cast<long long>(it->t  - t)))
            it = prev;
    }

    az = it->az;
    el = it->el;
    return true;
}
