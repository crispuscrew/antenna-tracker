#pragma once
#include <asio.hpp>
#include <string>
#include <array>
#include "config.h"
#include "tracking/tle_tracker.h"
#include "tracking/trajectory_planner.h"

class AntennaController {
public:
    AntennaController(asio::io_context& io, const Config& cfg);
    void start();

private:
    // ── State machine ─────────────────────────────────────────────────────────
    enum class State { WAITING, ACQUIRING, TRACKING, STOPPING };

    void transition(State next);
    void on_tick(const asio::error_code& ec);
    void schedule_tick();

    // ── Networking ────────────────────────────────────────────────────────────
    void connect();
    void do_read();
    void on_read(const asio::error_code& ec, std::size_t n);
    void send(std::string pkt); // принимает по значению — владение передаётся в shared_ptr

    // ── Members ───────────────────────────────────────────────────────────────
    asio::io_context&        io_;
    const Config&            cfg_;
    asio::ip::tcp::socket    socket_;
    asio::steady_timer       timer_;

    TleTracker               tracker_;
    TrajectoryPlanner        planner_;

    State                    state_   = State::WAITING;
    uint16_t                 seq_     = 0;
    Pass                     pass_;

    // Read ring-buffer.
    static constexpr std::size_t RX_BUF = 4096;
    std::array<char, RX_BUF>    rx_buf_{};
    std::string                  rx_accum_;

    bool                         debug_;
    bool                         pass_valid_ = false; // true когда pass_ актуален
};
