#include "antenna_controller.h"
#include "protocol/vkaprotocol.h"
#include "protocol/drive.h"
#include <iostream>
#include <iomanip>
#include <ctime>

// ── Constructor / start ───────────────────────────────────────────────────────

AntennaController::AntennaController(asio::io_context& io, const Config& cfg)
    : io_(io)
    , cfg_(cfg)
    , socket_(io)
    , timer_(io)
    , planner_(tracker_)
    , debug_(cfg.debug)
{
    if (!tracker_.load_tle(cfg.tle_file)) {
        throw std::runtime_error("Failed to load TLE from: " + cfg.tle_file);
    }
    tracker_.set_observer(cfg.lat_deg, cfg.lon_deg, cfg.alt_m);
}

void AntennaController::start()
{
    connect();
}

// ── Networking ────────────────────────────────────────────────────────────────

void AntennaController::connect()
{
    asio::ip::tcp::endpoint ep(
        asio::ip::make_address(cfg_.ip),
        static_cast<unsigned short>(cfg_.port));

    socket_.async_connect(ep, [this](const asio::error_code& ec) {
        if (ec) {
            std::cerr << "[connect] " << ec.message() << ", retrying in 5 s\n";
            timer_.expires_after(std::chrono::seconds(5));
            timer_.async_wait([this](const asio::error_code&) { connect(); });
            return;
        }
        std::cout << "[connected] " << cfg_.ip << ":" << cfg_.port << "\n";
        do_read();
        schedule_tick();
    });
}

void AntennaController::do_read()
{
    socket_.async_read_some(
        asio::buffer(rx_buf_),
        [this](const asio::error_code& ec, std::size_t n) { on_read(ec, n); });
}

void AntennaController::on_read(const asio::error_code& ec, std::size_t n)
{
    if (ec) {
        std::cerr << "[read] " << ec.message() << "\n";
        return;
    }

    rx_accum_.append(rx_buf_.data(), n);

    // Process all complete 30-byte packets present in the accumulator.
    while (rx_accum_.size() >= static_cast<std::size_t>(vka::PACKET_LEN)) {
        // Scan for the header '5555'.
        auto pos = rx_accum_.find("5555");
        if (pos == std::string::npos) {
            rx_accum_.clear();
            break;
        }
        if (pos > 0) {
            rx_accum_.erase(0, pos); // discard garbage before header
        }
        if (rx_accum_.size() < static_cast<std::size_t>(vka::PACKET_LEN))
            break; // wait for more data

        Drive d;
        if (vka::unpack(rx_accum_.c_str(), vka::PACKET_LEN, d)) {
            if (debug_) {
                std::cout << "[rx] seq=" << d.seq
                          << " state=" << static_cast<int>(d.state)
                          << " az=" << std::fixed << std::setprecision(2) << d.az
                          << " el=" << d.el << "\n";
            }
        }
        rx_accum_.erase(0, vka::PACKET_LEN);
    }

    // Keep accumulator bounded.
    if (rx_accum_.size() > RX_BUF * 2)
        rx_accum_.erase(0, rx_accum_.size() - RX_BUF);

    do_read();
}

void AntennaController::send(const std::string& pkt)
{
    if (!socket_.is_open()) return;
    // Fire-and-forget async write (pkt copied into lambda).
    asio::async_write(socket_, asio::buffer(pkt),
        [pkt, this](const asio::error_code& ec, std::size_t) {
            if (ec) std::cerr << "[write] " << ec.message() << "\n";
        });
}

// ── Timer / state machine ─────────────────────────────────────────────────────

void AntennaController::schedule_tick()
{
    timer_.expires_after(std::chrono::milliseconds(100));
    timer_.async_wait([this](const asio::error_code& ec) { on_tick(ec); });
}

void AntennaController::transition(State next)
{
    const char* names[] = {"WAITING", "ACQUIRING", "TRACKING", "STOPPING"};
    std::cout << "[state] " << names[static_cast<int>(state_)]
              << " -> " << names[static_cast<int>(next)] << "\n";
    state_ = next;
}

void AntennaController::on_tick(const asio::error_code& ec)
{
    if (ec) return; // timer cancelled

    std::time_t now = std::time(nullptr);

    switch (state_) {
    case State::WAITING: {
        // Plan the next pass.
        pass_ = planner_.plan_next(now);
        std::time_t prepoint_t = pass_.aos - 60;

        if (debug_) {
            char aos_str[32];
            struct tm* tm_info = gmtime(&pass_.aos);
            strftime(aos_str, sizeof(aos_str), "%Y-%m-%dT%H:%M:%SZ", tm_info);
            std::cout << "[pass] AOS=" << aos_str
                      << "  points=" << pass_.trajectory.size() << "\n";
        }

        if (now >= prepoint_t) {
            transition(State::ACQUIRING);
        }
        break;
    }

    case State::ACQUIRING: {
        // Pre-point to the first trajectory point.
        if (!pass_.trajectory.empty()) {
            const auto& first = pass_.trajectory.front();
            std::string pkt = vka::pack(seq_++, DriveCmd::TRACKER, first.az, first.el);
            send(pkt);
            if (debug_) {
                std::cout << "[acquiring] az=" << first.az << " el=" << first.el << "\n";
            }
        }
        if (now >= pass_.aos) {
            transition(State::TRACKING);
        }
        break;
    }

    case State::TRACKING: {
        float az = 0.f, el = 0.f;
        if (TrajectoryPlanner::lookup(pass_, now, az, el)) {
            std::string pkt = vka::pack(seq_++, DriveCmd::TRACKER, az, el);
            send(pkt);
            if (debug_) {
                std::cout << "[tracking] az=" << std::fixed << std::setprecision(2)
                          << az << " el=" << el << "\n";
            }
        }
        if (now >= pass_.los) {
            transition(State::STOPPING);
        }
        break;
    }

    case State::STOPPING: {
        // Send a stop command and return to WAITING.
        std::string pkt = vka::pack(seq_++, DriveCmd::STOP, 0.f, 0.f);
        send(pkt);
        transition(State::WAITING);
        break;
    }
    }

    schedule_tick();
}
