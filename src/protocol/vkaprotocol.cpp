#include "vkaprotocol.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cassert>

namespace vka {

// ── Helpers ───────────────────────────────────────────────────────────────────

uint8_t compute_lrc(const char* begin, const char* end)
{
    unsigned sum = 0;
    for (const char* p = begin; p != end; ++p)
        sum += static_cast<unsigned char>(*p);
    return static_cast<uint8_t>((256u - (sum % 256u)) % 256u);
}

int format_angle(char* buf, float deg)
{
    // Clamp to ±999.99 just in case.
    if (deg >  999.99f) deg =  999.99f;
    if (deg < -999.99f) deg = -999.99f;

    char sign = (deg < 0.f) ? '-' : '+';
    float abs_deg = std::fabs(deg);
    int   whole   = static_cast<int>(abs_deg);
    int   cents   = static_cast<int>(std::round((abs_deg - whole) * 100.f));
    if (cents >= 100) { cents -= 100; whole += 1; }

    return std::snprintf(buf, 7, "%c%03d%02d", sign, whole, cents);
}

float parse_angle(const char* p)
{
    // Format: ±DDDCC  (sign, 3 digits, 2 digits hundredths)
    char sign_ch = p[0];
    char tmp[8];
    std::memcpy(tmp, p + 1, 5);
    tmp[5] = '\0';

    int whole = (tmp[0] - '0') * 100 + (tmp[1] - '0') * 10 + (tmp[2] - '0');
    int cents = (tmp[3] - '0') * 10 + (tmp[4] - '0');

    float val = whole + cents / 100.0f;
    return (sign_ch == '-') ? -val : val;
}

// ── Encoding ──────────────────────────────────────────────────────────────────

// Response layout (30 bytes):
//   5555 | seq(2) | state(4) | az(6) | el(6) | reserve(4) | crc(2) | \r\n
//   0      4        6          10      16       22            26      28
//
// Command layout (30 bytes):
//   5555 | seq(2) | cmd(2) | data1=000000(6) | az(6) | el(6) | crc(2) | \r\n
//   0      4        6        8                 14      20       26       28

std::string pack(uint16_t seq, DriveCmd cmd, float az, float el)
{
    char buf[32] = {};

    // Header
    std::memcpy(buf, "5555", 4);

    // Sequence number (2 chars, decimal, zero-padded, wraps at 100)
    std::snprintf(buf + 4, 3, "%02u", static_cast<unsigned>(seq % 100u));

    // Command code (2 chars)
    std::snprintf(buf + 6, 3, "%02d", static_cast<int>(cmd));

    // data1 — always "000000" (6 chars)
    std::memcpy(buf + 8, "000000", 6);

    // Azimuth and elevation angles (6 chars each)
    char az_buf[8], el_buf[8];
    format_angle(az_buf, az);
    format_angle(el_buf, el);
    std::memcpy(buf + 14, az_buf, 6);
    std::memcpy(buf + 20, el_buf, 6);

    // CRC covers bytes [4, 26)  (seq through el, excluding header 5555 and \r\n)
    uint8_t lrc = compute_lrc(buf + 4, buf + 26);
    std::snprintf(buf + 26, 3, "%02X", static_cast<unsigned>(lrc));

    // Terminator
    buf[28] = '\r';
    buf[29] = '\n';

    return std::string(buf, PACKET_LEN);
}

// ── Decoding ──────────────────────────────────────────────────────────────────

bool unpack(const char* buf, std::size_t len, Drive& out)
{
    if (len < static_cast<std::size_t>(PACKET_LEN))
        return false;

    // Check header
    if (buf[0] != '5' || buf[1] != '5' || buf[2] != '5' || buf[3] != '5')
        return false;

    // Check terminator
    if (buf[28] != '\r' || buf[29] != '\n')
        return false;

    // CRC check (covers bytes [4, 26))
    uint8_t expected = compute_lrc(buf + 4, buf + 26);
    char crc_str[3] = { buf[26], buf[27], '\0' };
    unsigned received = 0;
    std::sscanf(crc_str, "%2X", &received);
    if (expected != static_cast<uint8_t>(received))
        return false;

    // Sequence
    char seq_str[3] = { buf[4], buf[5], '\0' };
    out.seq = static_cast<uint16_t>(std::atoi(seq_str));

    // State (4 chars at offset 6)
    char state_str[5] = {};
    std::memcpy(state_str, buf + 6, 4);
    out.state = static_cast<uint8_t>(std::atoi(state_str));

    // Azimuth (6 chars at offset 10)
    out.az = parse_angle(buf + 10);

    // Elevation (6 chars at offset 16)
    out.el = parse_angle(buf + 16);

    return true;
}

} // namespace vka
