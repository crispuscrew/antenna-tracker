#include "tle_tracker.h"
#include <predict/predict.h>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <cstring>

// Convert predict_julian_date_t back to Unix time_t.
// predict_julian_date_t = (unix_seconds - PREDICT_EPOCH) / 86400
// where PREDICT_EPOCH = 315302400  (1979-12-31 00:00:00 UTC as Unix seconds).
// Rather than hardcoding that constant we derive it from predict_to_julian(0).
static std::time_t jd_to_unix(predict_julian_date_t jd)
{
    // predict_to_julian(0) gives the Julian date for Unix epoch (t=0),
    // i.e. how many "predict days" separate 1979-12-31 from 1970-01-01.
    static const double jd_epoch = predict_to_julian(0);
    return static_cast<std::time_t>((jd - jd_epoch) * 86400.0);
}

TleTracker::~TleTracker()
{
    if (sat_) predict_destroy_orbital_elements(sat_);
    if (obs_) predict_destroy_observer(obs_);
}

bool TleTracker::load_tle(const std::string& path)
{
    std::ifstream f(path);
    if (!f) return false;

    std::string lines[3];
    int n = 0;
    std::string line;
    while (std::getline(f, line) && n < 3) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) lines[n++] = line;
    }

    // Accept 2-line TLE (without name) or 3-line (with name).
    const char* l1 = nullptr;
    const char* l2 = nullptr;
    if (n == 2) {
        l1 = lines[0].c_str();
        l2 = lines[1].c_str();
    } else if (n == 3) {
        l1 = lines[1].c_str();
        l2 = lines[2].c_str();
    } else {
        return false;
    }

    if (sat_) predict_destroy_orbital_elements(sat_);
    sat_ = predict_parse_tle(l1, l2);
    return sat_ != nullptr;
}

void TleTracker::set_observer(double lat_deg, double lon_deg, double alt_m)
{
    if (obs_) predict_destroy_observer(obs_);
    double lat_rad = lat_deg * M_PI / 180.0;
    double lon_rad = lon_deg * M_PI / 180.0;
    obs_ = predict_create_observer("gs", lat_rad, lon_rad, alt_m);
}

TleTracker::AzEl TleTracker::compute(std::time_t t) const
{
    predict_position orbit{};
    predict_observation obs_data{};

    predict_orbit(sat_, &orbit, predict_to_julian(t));
    predict_observe_orbit(obs_, &orbit, &obs_data);

    // azimuth and elevation are in radians.
    double az_deg = obs_data.azimuth  * 180.0 / M_PI;
    double el_deg = obs_data.elevation * 180.0 / M_PI;

    // Normalise azimuth to [0, 360).
    az_deg = std::fmod(az_deg, 360.0);
    if (az_deg < 0.0) az_deg += 360.0;

    return {az_deg, el_deg};
}

std::time_t TleTracker::next_aos(std::time_t from) const
{
    if (!sat_ || !obs_) return 0;
    predict_observation obs = predict_next_aos(obs_, sat_, predict_to_julian(from));
    return jd_to_unix(obs.time);
}

std::time_t TleTracker::next_los(std::time_t from) const
{
    if (!sat_ || !obs_) return 0;
    // Start searching from AOS of the next pass.
    predict_observation aos_obs = predict_next_aos(obs_, sat_, predict_to_julian(from));
    predict_observation los_obs = predict_next_los(obs_, sat_, aos_obs.time);
    return jd_to_unix(los_obs.time);
}
