#pragma once
#include <predict/predict.h>
#include <string>
#include <ctime>

class TleTracker {
public:
    TleTracker() = default;
    ~TleTracker();

    // Load TLE from file (expects exactly two lines, ignoring a possible
    // name-line at the top).  Returns false on error.
    bool load_tle(const std::string& path);

    // Set ground-station position.
    void set_observer(double lat_deg, double lon_deg, double alt_m);

    struct AzEl {
        double az_deg; // [0, 360)
        double el_deg; // [-90, 90]
    };

    // Compute az/el for the given Unix timestamp.
    AzEl compute(std::time_t t) const;

    // Return the next AOS after 'from' as a Unix timestamp.
    // Returns 0 if computation fails.
    std::time_t next_aos(std::time_t from) const;

    // Return the LOS following the next AOS after 'from'.
    std::time_t next_los(std::time_t from) const;

private:
    predict_orbital_elements_t* sat_ = nullptr;
    predict_observer_t*         obs_ = nullptr;
};
