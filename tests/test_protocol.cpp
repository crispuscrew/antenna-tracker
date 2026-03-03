// Unit-тесты протокола VKA (без Asio и libpredict, только stdlib).
// Запуск: ./build/test-protocol
// Или через CMake: ctest -V -R protocol

#include "protocol/vkaprotocol.h"
#include "protocol/drive.h"

#include <cstdio>
#include <cstring>
#include <cmath>

// ── Минималистичный тест-раннер ───────────────────────────────────────────────

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (cond) {                                                             \
            ++g_pass;                                                           \
        } else {                                                                \
            ++g_fail;                                                           \
            std::fprintf(stderr, "  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) < (eps))

static void section(const char* name)
{
    std::printf("\n── %s\n", name);
}

// ── Тесты ─────────────────────────────────────────────────────────────────────

static void test_lrc()
{
    section("compute_lrc");

    // Пустой диапазон → 0
    {
        char buf[1] = {};
        CHECK(vka::compute_lrc(buf, buf) == 0);
    }

    // Один байт 0xFF: (256 - 255) % 256 = 1
    {
        char buf[1] = { static_cast<char>(0xFF) };
        CHECK(vka::compute_lrc(buf, buf + 1) == 1);
    }

    // Один байт 0x00: (256 - 0) % 256 = 0
    {
        char buf[1] = { 0x00 };
        CHECK(vka::compute_lrc(buf, buf + 1) == 0);
    }

    // Два байта: 0x80 + 0x80 = 0x100 → sum % 256 = 0 → LRC = 0
    {
        char buf[2] = { static_cast<char>(0x80), static_cast<char>(0x80) };
        CHECK(vka::compute_lrc(buf, buf + 2) == 0);
    }

    // Известная последовательность: "01" = 0x30 + 0x31 = 0x61 = 97
    // LRC = (256 - 97) % 256 = 159 = 0x9F
    {
        const char* s = "01";
        CHECK(vka::compute_lrc(s, s + 2) == 159);
    }
}

static void test_format_angle()
{
    section("format_angle");

    char buf[8] = {};

    // 0°
    vka::format_angle(buf, 0.0f);
    CHECK(std::memcmp(buf, "+00000", 6) == 0);

    // +54.00°
    vka::format_angle(buf, 54.0f);
    CHECK(std::memcmp(buf, "+05400", 6) == 0);

    // -1.75°
    vka::format_angle(buf, -1.75f);
    CHECK(std::memcmp(buf, "-00175", 6) == 0);

    // +180.00°
    vka::format_angle(buf, 180.0f);
    CHECK(std::memcmp(buf, "+18000", 6) == 0);

    // -270.50°
    vka::format_angle(buf, -270.5f);
    CHECK(std::memcmp(buf, "-27050", 6) == 0);

    // +99.99°
    vka::format_angle(buf, 99.99f);
    CHECK(std::memcmp(buf, "+09999", 6) == 0);

    // +0.01°
    vka::format_angle(buf, 0.01f);
    CHECK(std::memcmp(buf, "+00001", 6) == 0);

    // -0.01°  (отрицательный ноль отображается как '-')
    vka::format_angle(buf, -0.01f);
    CHECK(std::memcmp(buf, "-00001", 6) == 0);
}

static void test_parse_angle()
{
    section("parse_angle");

    CHECK_NEAR(vka::parse_angle("+00000"), 0.0f,    0.005f);
    CHECK_NEAR(vka::parse_angle("+05400"), 54.0f,   0.005f);
    CHECK_NEAR(vka::parse_angle("-00175"), -1.75f,  0.005f);
    CHECK_NEAR(vka::parse_angle("+18000"), 180.0f,  0.005f);
    CHECK_NEAR(vka::parse_angle("-27050"), -270.5f, 0.005f);
    CHECK_NEAR(vka::parse_angle("+09999"), 99.99f,  0.005f);
}

static void test_angle_roundtrip()
{
    section("format/parse roundtrip");

    const float cases[] = {
        0.f, 1.f, -1.f, 45.f, -45.f, 90.f, 180.f,
        -180.f, 270.f, -250.f, 0.25f, -0.99f, 123.45f
    };

    for (float v : cases) {
        char buf[8] = {};
        vka::format_angle(buf, v);
        float back = vka::parse_angle(buf);
        CHECK_NEAR(back, v, 0.015f);
    }
}

static void test_pack()
{
    section("pack");

    // Длина пакета всегда 30 байт
    {
        auto pkt = vka::pack(0, DriveCmd::TRACKER, 0.f, 0.f);
        CHECK(static_cast<int>(pkt.size()) == vka::PACKET_LEN);
    }

    // Заголовок "5555"
    {
        auto pkt = vka::pack(1, DriveCmd::TRACKER, 10.f, 20.f);
        CHECK(pkt.substr(0, 4) == "5555");
    }

    // Терминатор \r\n
    {
        auto pkt = vka::pack(1, DriveCmd::TRACKER, 10.f, 20.f);
        CHECK(pkt[28] == '\r' && pkt[29] == '\n');
    }

    // Команда STOP → байты [6..7] = "03"
    {
        auto pkt = vka::pack(0, DriveCmd::STOP, 0.f, 0.f);
        CHECK(pkt[6] == '0' && pkt[7] == '3');
    }

    // Команда TRACKER → "02"
    {
        auto pkt = vka::pack(0, DriveCmd::TRACKER, 0.f, 0.f);
        CHECK(pkt[6] == '0' && pkt[7] == '2');
    }

    // seq оборачивается через 100
    {
        auto p99  = vka::pack(99,  DriveCmd::TRACKER, 0.f, 0.f);
        auto p100 = vka::pack(100, DriveCmd::TRACKER, 0.f, 0.f);
        // seq=99 → "99", seq=100 → "00"
        CHECK(p99.substr(4, 2)  == "99");
        CHECK(p100.substr(4, 2) == "00");
    }

    // CRC корректен: пересчитываем вручную
    {
        auto pkt = vka::pack(5, DriveCmd::TRACKER, 54.f, 30.f);
        uint8_t expected = vka::compute_lrc(pkt.c_str() + 4, pkt.c_str() + 26);
        char hex[3] = { pkt[26], pkt[27], '\0' };
        unsigned received = 0;
        std::sscanf(hex, "%2X", &received);
        CHECK(expected == static_cast<uint8_t>(received));
    }
}

static void test_unpack()
{
    section("unpack");

    // Пакет слишком короткий
    {
        Drive d;
        CHECK(!vka::unpack("5555", 4, d));
    }

    // Неверный заголовок
    {
        auto pkt = vka::pack(1, DriveCmd::TRACKER, 0.f, 0.f);
        // Сымитируем ответный пакет (layout немного другой, но заголовок тот же).
        // Просто испортим заголовок.
        pkt[0] = 'X';
        Drive d;
        CHECK(!vka::unpack(pkt.c_str(), pkt.size(), d));
    }

    // Неверный CRC
    {
        auto pkt = vka::pack(2, DriveCmd::TRACKER, 10.f, 5.f);
        pkt[26] ^= 0xFF; // испортить CRC
        Drive d;
        CHECK(!vka::unpack(pkt.c_str(), pkt.size(), d));
    }
}

static void test_pack_unpack_roundtrip()
{
    section("pack → unpack roundtrip (response layout)");

    // Собираем «ответный» пакет вручную (30 байт, response layout)
    // и проверяем, что unpack его корректно декодирует.
    //
    // Response layout:
    //   5555 | seq(2) | state(4) | az(6) | el(6) | reserve(4) | crc(2) | \r\n
    auto make_response = [](uint16_t seq, float az, float el) -> std::string {
        char buf[32] = {};
        std::memcpy(buf, "5555", 4);
        std::snprintf(buf + 4, 3, "%02u", static_cast<unsigned>(seq % 100u));
        std::memcpy(buf + 6, "0001", 4);  // state = "0001"
        char az_buf[8], el_buf[8];
        vka::format_angle(az_buf, az);
        vka::format_angle(el_buf, el);
        std::memcpy(buf + 10, az_buf, 6);
        std::memcpy(buf + 16, el_buf, 6);
        std::memcpy(buf + 22, "0000", 4); // reserve
        uint8_t lrc = vka::compute_lrc(buf + 4, buf + 26);
        std::snprintf(buf + 26, 3, "%02X", static_cast<unsigned>(lrc));
        buf[28] = '\r';
        buf[29] = '\n';
        return std::string(buf, vka::PACKET_LEN);
    };

    struct Case { uint16_t seq; float az; float el; };
    Case cases[] = {
        { 0,  0.f,    0.f   },
        { 7,  54.0f,  30.0f },
        { 42, -90.0f, 45.0f },
        { 99, 270.5f,  0.5f },
    };

    for (auto& c : cases) {
        auto pkt = make_response(c.seq, c.az, c.el);
        Drive d;
        bool ok = vka::unpack(pkt.c_str(), pkt.size(), d);
        CHECK(ok);
        if (ok) {
            CHECK(d.seq == c.seq % 100u);
            CHECK_NEAR(d.az, c.az, 0.015f);
            CHECK_NEAR(d.el, c.el, 0.015f);
        }
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test-protocol ===\n");

    test_lrc();
    test_format_angle();
    test_parse_angle();
    test_angle_roundtrip();
    test_pack();
    test_unpack();
    test_pack_unpack_roundtrip();

    std::printf("\nРезультат: %d прошли, %d упали\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
