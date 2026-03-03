#include <iostream>
#include <stdexcept>
#include <cstring>
#include <asio.hpp>
#include "config.h"
#include "controller/antenna_controller.h"

static void print_usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [OPTIONS]\n"
        << "\n"
        << "Required:\n"
        << "  --ip   <address>   Controller IP address\n"
        << "  --port <port>      TCP port          (default: 4001)\n"
        << "  --lat  <deg>       Observer latitude  (decimal degrees)\n"
        << "  --lon  <deg>       Observer longitude (decimal degrees)\n"
        << "  --alt  <metres>    Observer altitude  (default: 0)\n"
        << "  --tle  <file>      Path to TLE file (2 or 3 lines)\n"
        << "\n"
        << "Optional:\n"
        << "  --debug            Enable verbose logging\n"
        << "  -h / --help        Show this message\n";
}

static Config parse_args(int argc, char* argv[])
{
    Config cfg;
    bool have_ip  = false;
    bool have_lat = false;
    bool have_lon = false;
    bool have_tle = false;

    for (int i = 1; i < argc; ++i) {
        auto eq = [&](const char* flag) {
            return std::strcmp(argv[i], flag) == 0;
        };
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << argv[i] << "\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if (eq("--ip"))         { cfg.ip      = next(); have_ip  = true; }
        else if (eq("--port"))  { cfg.port    = std::atoi(next()); }
        else if (eq("--lat"))   { cfg.lat_deg = std::atof(next()); have_lat = true; }
        else if (eq("--lon"))   { cfg.lon_deg = std::atof(next()); have_lon = true; }
        else if (eq("--alt"))   { cfg.alt_m   = std::atof(next()); }
        else if (eq("--tle"))   { cfg.tle_file = next(); have_tle = true; }
        else if (eq("--debug")) { cfg.debug   = true; }
        else if (eq("-h") || eq("--help")) { print_usage(argv[0]); std::exit(0); }
        else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            print_usage(argv[0]);
            std::exit(1);
        }
    }

    if (!have_ip || !have_lat || !have_lon || !have_tle) {
        std::cerr << "Error: --ip, --lat, --lon and --tle are required.\n\n";
        print_usage(argv[0]);
        std::exit(1);
    }

    return cfg;
}

int main(int argc, char* argv[])
{
    Config cfg = parse_args(argc, argv);

    std::cout << "antenna-tracker starting\n"
              << "  controller : " << cfg.ip << ":" << cfg.port << "\n"
              << "  observer   : lat=" << cfg.lat_deg
                                << " lon=" << cfg.lon_deg
                                << " alt=" << cfg.alt_m << " m\n"
              << "  TLE file   : " << cfg.tle_file << "\n"
              << "  debug      : " << (cfg.debug ? "yes" : "no") << "\n";

    try {
        asio::io_context io;

        // Keep the io_context alive even when no async ops are pending.
        asio::executor_work_guard<asio::io_context::executor_type>
            work = asio::make_work_guard(io);

        AntennaController ctrl(io, cfg);
        ctrl.start();

        io.run();
    } catch (const std::exception& ex) {
        std::cerr << "Fatal: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
