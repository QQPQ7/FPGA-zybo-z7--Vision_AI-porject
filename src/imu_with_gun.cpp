#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

#include "imu.hpp"
#include "imu_data.h"

// ==================== 전역 플래그 ====================
static std::atomic<bool> g_running{true};
static std::mutex g_print_mtx;

static void on_sigint(int) {
    g_running = false;
}

// ==================== GPIO sysfs helper들 ====================

// 간단한 파일 쓰기 유틸
static int write_file(const std::string& path, const std::string& s) {
    std::ofstream f(path);
    if (!f) {
        std::perror(("open " + path).c_str());
        return -1;
    }
    f << s;
    if (!f.good()) {
        std::fprintf(stderr, "write failed: %s\n", path.c_str());
        return -1;
    }
    return 0;
}

static bool path_exists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0;
}

// sysfs GPIO helpers
static int gpio_export(int pin) {
    const std::string gp = "/sys/class/gpio/gpio" + std::to_string(pin);
    if (path_exists(gp)) return 0; // already exported
    return write_file("/sys/class/gpio/export", std::to_string(pin));
}

static int gpio_unexport(int pin) {
    const std::string gp = "/sys/class/gpio/gpio" + std::to_string(pin);
    if (!path_exists(gp)) return 0;
    return write_file("/sys/class/gpio/unexport", std::to_string(pin));
}

static int gpio_set_dir(int pin, bool output) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    return write_file(p, output ? "out" : "in");
}

static int gpio_set_active_low(int pin, bool active_low) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/active_low";
    if (!path_exists(p)) return 0; // 커널 옵션에 따라 없을 수 있음 → 무시
    return write_file(p, active_low ? "1" : "0");
}

static int gpio_write(int pin, int v) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    return write_file(p, v ? "1" : "0");
}

// ==================== 메인 ====================
int main() {
    std::signal(SIGINT, on_sigint);

    // ---------- IMU 초기화 ----------
    IMUReader imu_head("Head");
    imu_head.setDevice("/dev/i2c-0");

    if (!imu_head.init(true)) {
        std::fprintf(stderr, "[MAIN] IMU init failed\n");
        return 1;
    }

    // 출력 버퍼링 줄이기 (즉시 출력)
    setvbuf(stdout, nullptr, _IONBF, 0);

    constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;

    // ---------- GPIO 515 초기화 ----------
    const int GPIO_PIN = 515;

    if (gpio_export(GPIO_PIN) < 0) {
        std::fprintf(stderr, "[MAIN] gpio_export(%d) failed\n", GPIO_PIN);
        return 1;
    }

    // sysfs 노드가 만들어질 때까지 대기
    for (int i = 0; i < 50 &&
         !path_exists("/sys/class/gpio/gpio" + std::to_string(GPIO_PIN)); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (gpio_set_dir(GPIO_PIN, true) < 0) {
        std::fprintf(stderr, "[MAIN] gpio_set_dir(%d) failed\n", GPIO_PIN);
        return 1;
    }

    // active_low 사용 안 함 (필요하면 true로 변경 가능)
    (void)gpio_set_active_low(GPIO_PIN, false);

    // 시작 시 OFF
    (void)gpio_write(GPIO_PIN, 0);

    // ---------- IMU 읽기 스레드 ----------
    std::thread th([&](){
        imu_head.run_loop(200.0); // 200 Hz 루프
    });

    // 커서 숨김
    std::fputs("\x1b[?25l", stdout);
    std::puts("[MAIN] ICM-20948 + GPIO515 toggle every 5s... (Ctrl+C to quit)");

    // ---------- GPIO 토글 상태 ----------
    bool gpio_state = false;
    auto last_toggle = std::chrono::steady_clock::now();

    // ---------- 메인 루프 ----------
    while (g_running.load()) {
        IMUData d{};
        double ts = 0.0;
        uint64_t seq = 0;

        if (imu_head.latest(d, ts, &seq)) {
            // 필터 결과 (deg)
            double roll_filt  = d.mAngles[0];
            double pitch_filt = d.mAngles[1];
            double yaw_filt   = d.mAngles[2];

            // IMU 값 콘솔 출력
            std::cout << "Roll: " << roll_filt
                      << "  Pitch: " << pitch_filt
                      << "  Yaw: " << yaw_filt << "\n";
        }

        // ====== 5초마다 GPIO 토글 ======
        auto now = std::chrono::steady_clock::now();
        if (now - last_toggle >= std::chrono::seconds(5)) {
            last_toggle = now;
            gpio_state = !gpio_state;
            (void)gpio_write(GPIO_PIN, gpio_state ? 1 : 0);
            std::printf("[GPIO] gpio%d -> %s\n",
                        GPIO_PIN,
                        gpio_state ? "ON" : "OFF");
        }

        // 너무 바쁘지 않게 약간 쉼
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (th.joinable()) th.join();

    // 종료 시 GPIO 끄고 unexport (선택)
    (void)gpio_write(GPIO_PIN, 0);
    (void)gpio_unexport(GPIO_PIN);

    // 커서 복원
    std::fputs("\x1b[?25h\n", stdout);
    std::puts("[MAIN] bye.");
    return 0;
}
