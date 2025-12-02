#pragma once
#include <cstdint>
#include <cstdio>

class ServoPWM {
private:
    // Servo constants
    static constexpr uint32_t SERVO_MIN_US = 500;   // 1.0 ms
    static constexpr uint32_t SERVO_MAX_US = 2500;  // 2.0 ms
    static constexpr uint32_t SERVO_NEUTRAL_US = 1500; // 1.5 ms

    int chip_;
    int channel_;

    // Helper function to write to sysfs
    static int write_u64(const char* path, unsigned long long v) {
        if (FILE* f = ::fopen(path, "w")) {
            int r = ::fprintf(f, "%llu", v);
            ::fclose(f);
            return r < 0 ? -1 : 0;
        }
        return -1;
    }

    // Helper function to construct sysfs paths
    static void mk_path(char* buf, size_t n, int chip, const char* tail, int ch = -1) {
        if (ch == -1) {
            ::snprintf(buf, n, "/sys/class/pwm/pwmchip%d/%s", chip, tail);
        } else {
            ::snprintf(buf, n, "/sys/class/pwm/pwmchip%d/pwm%d/%s", chip, ch, tail);
        }
    }

public:
    ServoPWM() : chip_(-1), channel_(-1) {}
    
    // Initialize the PWM for a specific chip and channel
    int init(int chip, int ch, uint32_t period_ns) {
        chip_ = chip;
        channel_ = ch;
        char p[192];

        // Export (ignore if already exported)
        mk_path(p, sizeof(p), chip_, "export");
        (void)write_u64(p, static_cast<unsigned long long>(channel_));

        // Safely disable
        mk_path(p, sizeof(p), chip_, "enable", channel_);
        (void)write_u64(p, 0);

        // Set period -> duty (neutral) -> enable
        mk_path(p, sizeof(p), chip_, "period", channel_);
        if (write_u64(p, static_cast<unsigned long long>(period_ns)) < 0) return -1;

        mk_path(p, sizeof(p), chip_, "duty_cycle", channel_);
        if (write_u64(p, static_cast<unsigned long long>(SERVO_NEUTRAL_US) * 1000ULL) < 0) return -1;

        mk_path(p, sizeof(p), chip_, "enable", channel_);
        if (write_u64(p, 1) < 0) return -1;

        return 0;
    }

    // Write PWM pulse width in microseconds
    int write_us(uint32_t usec) {
        if (usec < SERVO_MIN_US) usec = SERVO_MIN_US;
        if (usec > SERVO_MAX_US) usec = SERVO_MAX_US;
        char p[192];
        mk_path(p, sizeof(p), chip_, "duty_cycle", channel_);
        return write_u64(p, static_cast<unsigned long long>(usec) * 1000ULL);
    }

    // Set servo angle in degrees
    int set_angle_deg(float deg, float min_deg, float max_deg, float offset, int dir) {
        // Apply offset and direction
        deg += offset;
        deg *= dir;
        if (deg < min_deg) deg = min_deg;
        if (deg > max_deg) deg = max_deg;

        const float span_us = static_cast<float>(SERVO_MAX_US - SERVO_MIN_US);
        const float us_f = static_cast<float>(SERVO_NEUTRAL_US)
                         + (deg / 90.f) * (span_us * 0.5f);
        return write_us(static_cast<uint32_t>(us_f));
    }

    // Deinitialize the PWM
    int deinit() {
        char p[192];
        mk_path(p, sizeof(p), chip_, "enable", channel_);
        (void)write_u64(p, 0);
        mk_path(p, sizeof(p), chip_, "unexport");
        return write_u64(p, static_cast<unsigned long long>(channel_));
    }
};