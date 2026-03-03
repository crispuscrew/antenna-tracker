#pragma once
#include <string>
#include <cstdint>
#include "drive.h"

namespace vka {

// Packet size (including trailing \r\n).
constexpr int PACKET_LEN = 30;

// ── Encoding ─────────────────────────────────────────────────────────────────

// Build a 30-byte command packet.
//   seq : sequence number [0, 99]
//   cmd : DriveCmd::TRACKER or DriveCmd::STOP
//   az  : commanded azimuth,   degrees (may be outside [0,360) due to unwrap)
//   el  : commanded elevation, degrees [0, 90]
//
// Returns a std::string of exactly 30 bytes ending with \r\n, or empty on error.
std::string pack(uint16_t seq, DriveCmd cmd, float az, float el);

// ── Decoding ─────────────────────────────────────────────────────────────────

// Parse a 30-byte response packet from the controller into a Drive struct.
// Returns true on success (valid header, length, and CRC).
bool unpack(const char* buf, std::size_t len, Drive& out);

// ── Helpers (exposed for unit-testing) ───────────────────────────────────────

// Compute LRC over bytes [begin, end).
// LRC = (256 - (sum % 256)) % 256
uint8_t compute_lrc(const char* begin, const char* end);

// Format a signed angle to ±DDDCC format (6 chars, no null terminator needed).
// Returns number of chars written (always 6) into buf (must hold at least 7).
int format_angle(char* buf, float deg);

// Parse an angle from ±DDDCC format.  Returns degrees as float.
float parse_angle(const char* p);

} // namespace vka
