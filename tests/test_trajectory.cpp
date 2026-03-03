// Unit-тесты TrajectoryPlanner::lookup() и логики выбора ближайшей точки.
// Не использует libpredict напрямую (plan_next() не вызывается).
// Запуск: ./build/test-trajectory  /  ctest -V -R trajectory

#include "tracking/trajectory_planner.h"
#include <cstdio>
#include <cmath>
#include <vector>

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (cond) { ++g_pass; }                                                  \
        else {                                                                   \
            ++g_fail;                                                            \
            std::fprintf(stderr, "  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((double)(a) - (double)(b)) < (eps))

static void section(const char* name) { std::printf("\n── %s\n", name); }

// Вспомогательная функция: строит Pass с заданными точками
static Pass make_pass(std::time_t t0, std::vector<std::pair<float,float>> points)
{
    Pass p;
    p.aos = t0;
    p.los = t0 + static_cast<std::time_t>(points.size()) - 1;
    for (std::size_t i = 0; i < points.size(); ++i) {
        p.trajectory.push_back({
            t0 + static_cast<std::time_t>(i),
            points[i].first,
            points[i].second
        });
    }
    return p;
}

// ── Тесты lookup() ────────────────────────────────────────────────────────────

static void test_lookup_empty()
{
    section("lookup: пустая траектория");
    Pass p;
    float az = -1.f, el = -1.f;
    CHECK(!TrajectoryPlanner::lookup(p, 1000, az, el));
}

static void test_lookup_single_point()
{
    section("lookup: одна точка");
    Pass p = make_pass(1000, {{ 90.f, 30.f }});
    float az = 0.f, el = 0.f;
    CHECK(TrajectoryPlanner::lookup(p, 1000, az, el));
    CHECK_NEAR(az, 90.f, 0.01f);
    CHECK_NEAR(el, 30.f, 0.01f);
    // До и после единственной точки — тоже она
    CHECK(TrajectoryPlanner::lookup(p, 999, az, el));
    CHECK_NEAR(az, 90.f, 0.01f);
    CHECK(TrajectoryPlanner::lookup(p, 1001, az, el));
    CHECK_NEAR(az, 90.f, 0.01f);
}

static void test_lookup_exact()
{
    section("lookup: точное совпадение по времени");
    Pass p = make_pass(1000, {
        {  10.f, 5.f },
        {  20.f, 10.f },
        {  30.f, 15.f },
        {  40.f, 20.f },
    });
    float az, el;
    TrajectoryPlanner::lookup(p, 1000, az, el); CHECK_NEAR(az, 10.f, 0.01f);
    TrajectoryPlanner::lookup(p, 1001, az, el); CHECK_NEAR(az, 20.f, 0.01f);
    TrajectoryPlanner::lookup(p, 1002, az, el); CHECK_NEAR(az, 30.f, 0.01f);
    TrajectoryPlanner::lookup(p, 1003, az, el); CHECK_NEAR(az, 40.f, 0.01f);
}

static void test_lookup_nearest()
{
    section("lookup: ближайшая точка (между двумя)");
    // Точки в t=1000 и t=1002, ищем t=1001 — одинаково близко к обеим,
    // должна вернуться одна из двух (не аварийно).
    Pass p = make_pass(1000, {
        { 100.f, 10.f },
        { 110.f, 15.f },  // t=1001
        { 120.f, 20.f },  // t=1002
    });
    float az, el;
    // Ровно между t=1000 и t=1002 — вернёт одну из крайних, не краш
    bool ok = TrajectoryPlanner::lookup(p, 1001, az, el);
    CHECK(ok);
    CHECK(std::fabs(az - 110.f) < 0.01f); // должна быть точка t=1001

    // Чуть ближе к первой точке: t=1000 + 0.4 с (но time_t целый, т.е. t=1000)
    TrajectoryPlanner::lookup(p, 1000, az, el);
    CHECK_NEAR(az, 100.f, 0.01f);
}

static void test_lookup_clamp()
{
    section("lookup: запрос до/после траектории");
    Pass p = make_pass(1000, {
        { 45.f, 5.f },
        { 90.f, 30.f },
        { 135.f, 60.f },
    });
    float az, el;
    // До начала → первая точка
    TrajectoryPlanner::lookup(p, 500, az, el);
    CHECK_NEAR(az, 45.f, 0.01f);

    // После конца → последняя точка
    TrajectoryPlanner::lookup(p, 9999, az, el);
    CHECK_NEAR(az, 135.f, 0.01f);
}

static void test_azimuth_unwrap_manual()
{
    section("раскрутка азимута: пример вычисления вручную");
    // Проверяем алгоритм unwrap напрямую, без TleTracker.
    // Алгоритм из trajectory_planner.cpp:
    //   offsets = {0, +360, -360, -720}
    //   выбрать тот, при котором |raw + offset - prev| минимален

    auto unwrap = [](float raw, float prev) -> float {
        const float offsets[4] = {0.f, 360.f, -360.f, -720.f};
        float best = raw;
        float min_dist = std::fabs(raw - prev);
        for (float off : offsets) {
            float candidate = raw + off;
            float dist = std::fabs(candidate - prev);
            if (dist < min_dist) {
                min_dist = dist;
                best = candidate;
            }
        }
        return best;
    };

    // Нет скачка (prev=90, raw=100 → 100)
    CHECK_NEAR(unwrap(100.f, 90.f), 100.f, 0.01f);

    // Пересечение 0° снизу вверх: prev=350, raw=10 → должен дать 370
    CHECK_NEAR(unwrap(10.f, 350.f), 370.f, 0.01f);

    // Пересечение 360° сверху вниз: prev=10, raw=350 → должен дать -10
    CHECK_NEAR(unwrap(350.f, 10.f), -10.f, 0.01f);

    // Большой скачок назад: prev=270, raw=5 → ближе через -355 (5-360=-355) или +5?
    // |5 - 270| = 265, |5+360 - 270| = 95, |5-360 - 270| = 625
    // → best = 365
    CHECK_NEAR(unwrap(5.f, 270.f), 365.f, 0.01f);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main()
{
    std::printf("=== test-trajectory ===\n");

    test_lookup_empty();
    test_lookup_single_point();
    test_lookup_exact();
    test_lookup_nearest();
    test_lookup_clamp();
    test_azimuth_unwrap_manual();

    std::printf("\nРезультат: %d прошли, %d упали\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
