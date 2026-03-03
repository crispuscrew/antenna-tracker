#pragma once
#include <string>

struct Config {
    std::string ip;
    int         port    = 4001;
    double      lat_deg = 0.0;   // geodetic latitude,  degrees
    double      lon_deg = 0.0;   // geodetic longitude, degrees
    double      alt_m   = 0.0;   // altitude above sea level, metres
    std::string tle_file;
    bool        debug   = false;
};
