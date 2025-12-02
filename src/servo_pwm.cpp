// src/servo_pwm.cpp
#include "servo_pwm.hpp"
#include <cstdio>
#include <cstdint>

// ===== 서보 상수(필요하면 여기서만 바꿔) =====
static constexpr uint32_t SERVO_MIN_US      = 500;       // 1.0 ms
static constexpr uint32_t SERVO_MAX_US      = 2500;       // 2.0 ms
static constexpr uint32_t SERVO_NEUTRAL_US  = 1500;       // 1.5 ms

namespace {
int write_u64(const char* path, unsigned long long v) {
    if (FILE* f = ::fopen(path, "w")) {
        int r = ::fprintf(f, "%llu", v);
        ::fclose(f);
        return r < 0 ? -1 : 0;
    }
    return -1;
}
void mk(char* buf, size_t n, int chip, const char* tail) {
    ::snprintf(buf, n, "/sys/class/pwm/pwmchip%d/%s", chip, tail);
}
} // anon

namespace servo {

int init(int chip, int ch, uint32_t period_ns) {
    char p[192];

    // export (이미 export돼 있으면 무시)
    mk(p, sizeof p, chip, "export");
    (void)write_u64(p, static_cast<unsigned long long>(ch));

    // 안전하게 disable
    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/enable", chip, ch);
    (void)write_u64(p, 0);

    // period -> duty(neutral) -> enable
    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/period", chip, ch);
    if (write_u64(p, static_cast<unsigned long long>(period_ns)) < 0) return -1;

    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", chip, ch);
    if (write_u64(p, static_cast<unsigned long long>(SERVO_NEUTRAL_US) * 1000ULL) < 0) return -1;

    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/enable", chip, ch);
    if (write_u64(p, 1) < 0) return -1;

    return 0;
}

int write_us(int chip, int ch, uint32_t usec) {
    if (usec < SERVO_MIN_US) usec = SERVO_MIN_US;
    if (usec > SERVO_MAX_US) usec = SERVO_MAX_US;

    char p[192];
    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", chip, ch);
    return write_u64(p, static_cast<unsigned long long>(usec) * 1000ULL);
}

int set_angle_deg(int chip, int ch, float deg, float min_deg, float max_deg, float offset, int dir) {
    // 방향에 따라 각도를 반전시킴
    deg += offset;

    // 방향이 반대일 경우 각도를 반전
    deg *= dir;

    if (deg < min_deg) deg = min_deg;
    if (deg > max_deg) deg = max_deg;

    const float span_us = static_cast<float>(SERVO_MAX_US - SERVO_MIN_US);
    const float us_f = static_cast<float>(SERVO_NEUTRAL_US)
                     + (deg / 90.f) * (span_us * 0.5f);

    return write_us(chip, ch, static_cast<uint32_t>(us_f));
}

int deinit(int chip, int ch) {
    char p[192];
    ::snprintf(p, sizeof p, "/sys/class/pwm/pwmchip%d/pwm%d/enable", chip, ch);
    (void)write_u64(p, 0);

    mk(p, sizeof p, chip, "unexport");
    return write_u64(p, static_cast<unsigned long long>(ch));
}

} // namespace servo